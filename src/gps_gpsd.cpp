/*
 * GPSD client class
 *
 * (c) 2026 Erik Tkal
 *
 */

#include "gps_gpsd.h"

#include <algorithm>
#include <iostream>

#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "network_info.h"

namespace
{
    constexpr uint8_t pollTimeSeconds = 5;
    constexpr char gpsdWatchCommand[] = "?WATCH={\"nmea\":true}\r\n";
} // namespace

GPS_gpsd::GPS_gpsd()
{
}

GPS_gpsd::~GPS_gpsd()
{
    closeConnection();
    queue_free(&m_qSentences); // Free the queue resources
}

void GPS_gpsd::Initialize()
{
    GPS::Initialize();

    // Initialize the queue for received sentences. This uses the SDK queue_t structure for thread-safe access.
    queue_init(&m_qSentences, GPS_BUFSIZE, GPS_QUEUE_SIZE); // Initialize the queue for received sentences
    m_errorSleepMs = GPSD_ERROR_SLEEP_INITIAL_MS;
}

bool GPS_gpsd::wifiStateMachine()
{
    // Poll the lwip stack
    cyw43_arch_poll();

    err_t err = ERR_OK;
    switch (m_wifiState)
    {
    case WifiState::DISCONNECTED:
        // Attempt to connect
        LogInfo("Wifi disconnected, initializing connection");
        cyw43_arch_enable_sta_mode();
        cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM & ~0xf);
        SetWifiState(WifiState::CONNECTING);
        std::cout << "Calling cyw43_arch_wifi_connect_async" << std::endl;
        cyw43_arch_wifi_connect_async(g_szWifiSsid, g_szWifiPassword, CYW43_AUTH_WPA2_AES_PSK);
        m_wifiConnectTimeout = make_timeout_time_ms(5000);
        return false;

    case WifiState::CONNECTING:
        if (CYW43_LINK_UP == cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA))
        {
            LogInfo("Wifi connected successfully");
            SetWifiState(WifiState::CONNECTED);
            return false;
        }
        if (CYW43_LINK_BADAUTH == cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA))
        {
            LogInfo("Wifi authentication failed");
            SetWifiState(WifiState::DISCONNECTED);
            return false;
        }
        sleep_ms(10);
        if (absolute_time_diff_us(get_absolute_time(), m_wifiConnectTimeout) < 0)
        {
            // Handle timeout error
            cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
            sleep_ms(500);
            std::cout << "Failed to connect to Wifi; retrying" << std::endl;
            SetWifiState(WifiState::DISCONNECTED);
        }
        return false;

    case WifiState::CONNECTED:
        switch (m_gpsdServerState)
        {
        case GpsdServerState::DISCONNECTED:
            ip4addr_aton(g_szGpsdIpAddress, &m_remoteAddr);
            std::cout << "Preparing connection to " << ip4addr_ntoa(&m_remoteAddr) << " port " << g_nGpsdTcpPort << std::endl;
            m_pTcpPcb = tcp_new_ip_type(IP_GET_TYPE(m_remoteAddr));
            if (m_pTcpPcb == nullptr)
            {
                LogInfo("Unable to allocate TCP control block for gpsd");
                panic("Unable to allocate TCP control block for gpsd");
                return false;
            }
            tcp_arg(m_pTcpPcb, this);
            tcp_poll(m_pTcpPcb, tcpPoll, pollTimeSeconds * 2);
            tcp_sent(m_pTcpPcb, tcpSent);
            tcp_recv(m_pTcpPcb, tcpRecv);
            tcp_err(m_pTcpPcb, tcpError);
            tcp_nagle_disable(m_pTcpPcb);
            SetServerState(GpsdServerState::CONNECTING);
            return false;

        case GpsdServerState::CONNECTING:
            LogInfo("Attempting to connect to gpsd server");
            cyw43_arch_lwip_begin();
            err = tcp_connect(m_pTcpPcb, &m_remoteAddr, g_nGpsdTcpPort, tcpConnected);
            cyw43_arch_lwip_end();
            if (err != ERR_OK)
            {
                LogInfo("Error connecting to gpsd: " + std::to_string(err));
                LogInfo("Setting GPSD server state to ERROR");
                SetServerState(GpsdServerState::ERROR);
                return false;
            }
            LogInfo("Waiting for tcp_connect result");
            SetServerState(GpsdServerState::WAITING_FOR_CONNECT_RESULT);
            return false;

        case GpsdServerState::WAITING_FOR_CONNECT_RESULT:
            return false;

        case GpsdServerState::CONNECTION_SUCCEEDED:
            LogInfo("Sending watch command: " + std::string(gpsdWatchCommand));
            writeString(gpsdWatchCommand);
            SetServerState(GpsdServerState::CONNECTED);
            m_errorSleepMs = GPSD_ERROR_SLEEP_INITIAL_MS;
            return true;

        case GpsdServerState::CONNECTED:
            return true;

        case GpsdServerState::ERROR:
            LogInfo("Gpsd server connection error");
            // A lapse in data reception has occurred, indicating a potential connection issue
            closeConnection();
            sleep_ms(m_errorSleepMs); // Wait before attempting to reconnect
            m_errorSleepMs = std::min(m_errorSleepMs + GPSD_ERROR_SLEEP_STEP_MS, GPSD_ERROR_SLEEP_MAX_MS);
            if (CYW43_LINK_UP == cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA))
            {
                // If the link is still up, reset the server connection
                SetServerState(GpsdServerState::DISCONNECTED);
            }
            else
            {
                // If the link is down, mark the Wi-Fi as disconnected
                cyw43_arch_disable_sta_mode();
                SetServerState(GpsdServerState::DISCONNECTED);
                SetWifiState(WifiState::DISCONNECTED);
            }
            return false;

        default:
            return false;
        }
    default:
        return false;
    }
    return false;
}

// Get a sentence from the queue. This function will return false if no sentence is available.
bool GPS_gpsd::getSentence(std::string& strSentence)
{
    if (!wifiStateMachine())
    {
        return false;
    }

    // Drain any bytes written to the circular buffer since our last pass.
    size_t nWritePos = m_iRingWritePos;
    size_t nReadPos = m_iRingReadPos;

    if (nWritePos != nReadPos)
    {
        if (nWritePos > nReadPos)
        {
            // Contiguous region: read [readPos, writePos)
            processRingBytes(reinterpret_cast<const uint8_t*>(m_szRingBuf) + nReadPos, nWritePos - nReadPos);
        }
        else
        {
            // Wrapped: read [readPos, end) then [0, writePos)
            processRingBytes(reinterpret_cast<const uint8_t*>(m_szRingBuf) + nReadPos, GPS_RING_BUFSIZE - nReadPos);
            processRingBytes(reinterpret_cast<const uint8_t*>(m_szRingBuf), nWritePos);
        }
        m_iRingReadPos = nWritePos;
    }

    bool bFound = false;

    // Check if there are any sentences in the queue. If so, remove one and return it.
    if (queue_try_peek(&m_qSentences, nullptr))
    {
        char szBuffer[GPS_BUFSIZE];
        if (queue_try_remove(&m_qSentences, szBuffer))
        {
            strSentence = std::string(szBuffer);
            bFound = true;
        }
    }
    return bFound;
}

void GPS_gpsd::closeConnection()
{
    if (m_pTcpPcb == nullptr)
    {
        return;
    }

    tcp_arg(m_pTcpPcb, nullptr);
    tcp_poll(m_pTcpPcb, nullptr, 0);
    tcp_sent(m_pTcpPcb, nullptr);
    tcp_recv(m_pTcpPcb, nullptr);
    tcp_err(m_pTcpPcb, nullptr);
    if (tcp_close(m_pTcpPcb) != ERR_OK)
    {
        tcp_abort(m_pTcpPcb);
    }
    m_pTcpPcb = nullptr;

    sleep_ms(1000); // Wait for a moment to ensure the connection is properly closed.
}

void GPS_gpsd::writeString(const std::string& str)
{
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(m_pTcpPcb, str.c_str(), str.length(), TCP_WRITE_FLAG_COPY);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        std::cout << "lwIP tcp_write error " << std::to_string(err) << std::endl;
    }
}

err_t GPS_gpsd::tcpConnected(void* arg, struct tcp_pcb* pcb, err_t err)
{
    return static_cast<GPS_gpsd*>(arg)->onTcpConnected(pcb, err);
}

err_t GPS_gpsd::tcpPoll(void* arg, struct tcp_pcb* pcb)
{
    return static_cast<GPS_gpsd*>(arg)->onTcpPoll(pcb);
}

err_t GPS_gpsd::tcpSent(void* arg, struct tcp_pcb* pcb, u16_t len)
{
    return static_cast<GPS_gpsd*>(arg)->onTcpSent(pcb, len);
}

err_t GPS_gpsd::tcpRecv(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err)
{
    return static_cast<GPS_gpsd*>(arg)->onTcpRecv(pcb, p, err);
}

void GPS_gpsd::tcpError(void* arg, err_t err)
{
    static_cast<GPS_gpsd*>(arg)->onTcpError(err);
}

err_t GPS_gpsd::onTcpConnected(struct tcp_pcb* pcb, err_t err)
{
    if (err != ERR_OK)
    {
        // Unsuccessful, set error in order to retry
        LogInfo("gpsd connection error " + std::to_string(err));
        LogInfo("Setting GPSD server state to ERROR");
        SetServerState(GpsdServerState::ERROR);
        return ERR_OK;
    }
    LogInfo("GPSD server connection established successfully.");
    SetServerState(GpsdServerState::CONNECTION_SUCCEEDED);
    return ERR_OK;
}

err_t GPS_gpsd::onTcpPoll(struct tcp_pcb* pcb)
{
    return ERR_OK;
}

err_t GPS_gpsd::onTcpSent(struct tcp_pcb* pcb, u16_t len)
{
    return ERR_OK;
}

err_t GPS_gpsd::onTcpRecv(struct tcp_pcb* pcb, struct pbuf* p, err_t err)
{
    cyw43_arch_lwip_check();
    if (p == nullptr)
    {
        LogInfo("Received null pbuf.");
        LogInfo("Setting GPSD server state to ERROR");
        SetServerState(GpsdServerState::ERROR);
        return ERR_OK;
    }

    for (pbuf* buffer = p; buffer != nullptr; buffer = buffer->next)
    {
        const char* payload = static_cast<const char*>(buffer->payload);
        for (uint16_t index = 0; index < buffer->len; ++index)
        {
            m_szRingBuf[m_iRingWritePos] = payload[index];
            m_iRingWritePos = (m_iRingWritePos + 1) % GPS_RING_BUFSIZE;
        }
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

void GPS_gpsd::onTcpError(err_t err)
{
    m_pTcpPcb = nullptr;
    LogInfo("gpsd TCP error " + std::to_string(err));
    LogInfo("Setting GPSD server state to ERROR");
    SetServerState(GpsdServerState::ERROR);
}

// Feed received bytes into the sentence assembly buffer. Completed sentences are queued for the main loop.
void GPS_gpsd::processRingBytes(const uint8_t* pBuf, size_t nLen)
{
    for (size_t i = 0; i < nLen; ++i)
    {
        char ch = static_cast<char>(pBuf[i]);
        if (ch == '$')
        {
            // Start of a new NMEA sentence
            m_iNext = 0;
            m_szBuf[m_iNext++] = ch;
        }
        else if (m_iNext > 0)
        {
            if (m_iNext < GPS_BUFSIZE - 1)
            {
                m_szBuf[m_iNext++] = ch;
            }
            else
            {
                // Overflow without newline; reset buffer
                m_iNext = 0;
            }

            if (ch == '\n')
            {
                m_szBuf[m_iNext] = '\0';
                if (!queue_try_add(&m_qSentences, m_szBuf))
                {
                    // Should never happen if the queue is sized appropriately
                    printf("Queue full\n");
                }
                m_iNext = 0;
            }
        }
    }
}

std::string GPS_gpsd::WifiStateToString(WifiState state)
{
    switch (state)
    {
    case WifiState::DISCONNECTED:
        return "DISCONNECTED";
    case WifiState::CONNECTING:
        return "CONNECTING";
    case WifiState::CONNECTED:
        return "CONNECTED";
    case WifiState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

std::string GPS_gpsd::GpsdServerStateToString(GpsdServerState state)
{
    switch (state)
    {
    case GpsdServerState::DISCONNECTED:
        return "DISCONNECTED";
    case GpsdServerState::CONNECTING:
        return "CONNECTING";
    case GpsdServerState::WAITING_FOR_CONNECT_RESULT:
        return "WAITING_FOR_CONNECT_RESULT";
    case GpsdServerState::CONNECTION_SUCCEEDED:
        return "CONNECTION_SUCCEEDED";
    case GpsdServerState::CONNECTED:
        return "CONNECTED";
    case GpsdServerState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
