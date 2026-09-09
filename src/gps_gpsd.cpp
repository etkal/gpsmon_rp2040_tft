/*
 * GPSD client class
 *
 * (c) 2026 Erik Tkal
 *
 */

#include "gps_gpsd.h"

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

    LogInfo("Connecting to Wi-Fi");
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM & ~0xf);
    // while (cyw43_arch_wifi_connect_timeout_ms(g_szWifiSsid, g_szWifiPassword, CYW43_AUTH_WPA2_AES_PSK, 5000))
    // {
    //     LogInfo("Failed to connect to Wi-Fi; retrying");
    //     cyw43_arch_poll();
    // }
    bool bConnected = false;
    while (!bConnected)
    {
        std::cout << "Calling cyw43_arch_wifi_connect_async" << std::endl;
        cyw43_arch_wifi_connect_async(g_szWifiSsid, g_szWifiPassword, CYW43_AUTH_WPA2_AES_PSK);
        absolute_time_t timeout = make_timeout_time_ms(5000);
        while (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
        {
            cyw43_arch_poll();
            sleep_ms(10);
            if (absolute_time_diff_us(get_absolute_time(), timeout) < 0)
            {
                // Handle timeout error
                std::cout << "Failed to connect to Wi-Fi; retrying" << std::endl;
                break;
            }
        }
        std::cout << "Calling cyw43_tcpip_link_status" << std::endl;
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP)
        {
            bConnected = true;
        }
    }
    LogInfo("Connected to Wi-Fi");

    ip4addr_aton(g_szGpsdIpAddress, &m_remoteAddr);
    std::cout << "Connecting to " << ip4addr_ntoa(&m_remoteAddr) << " port " << g_nGpsdTcpPort << std::endl;
    m_pTcpPcb = tcp_new_ip_type(IP_GET_TYPE(m_remoteAddr));
    if (m_pTcpPcb == nullptr)
    {
        LogInfo("Unable to allocate TCP control block for gpsd");
        Stop();
        return;
    }

    tcp_arg(m_pTcpPcb, this);
    tcp_poll(m_pTcpPcb, tcpPoll, pollTimeSeconds * 2);
    tcp_sent(m_pTcpPcb, tcpSent);
    tcp_recv(m_pTcpPcb, tcpRecv);
    tcp_err(m_pTcpPcb, tcpError);
    tcp_nagle_disable(m_pTcpPcb);

    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(m_pTcpPcb, &m_remoteAddr, g_nGpsdTcpPort, tcpConnected);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        std::cout << "Error connecting to gpsd: " << err << std::endl;
        Stop();
    }
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
}

void GPS_gpsd::writeString(const std::string& str)
{
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(m_pTcpPcb, str.c_str(), str.length(), TCP_WRITE_FLAG_COPY);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        std::cout << "lwIP tcp_write error " << err << std::endl;
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
        std::cout << "gpsd connection error " << err << std::endl;
        Stop();
        return ERR_OK;
    }

    LogInfo("Sending watch command: " + std::string(gpsdWatchCommand));
    writeString(gpsdWatchCommand);
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
        LogInfo("Received null pbuf, stopping GPS.");
        Stop();
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

void GPS_gpsd::onTcpError(err_t err)
{
    m_pTcpPcb = nullptr;
    std::cout << "gpsd TCP error " << err << std::endl;
    Stop();
}

// Get a sentence from the queue. This function will return false if no sentence is available.
bool GPS_gpsd::getSentence(std::string& strSentence)
{
    // Poll the lwip stack
    cyw43_arch_poll();

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
