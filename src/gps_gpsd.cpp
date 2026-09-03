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
    while (cyw43_arch_wifi_connect_timeout_ms(g_szWifiSsid, g_szWifiPassword, CYW43_AUTH_WPA2_AES_PSK, 5000))
    {
        LogInfo("Failed to connect to Wi-Fi; retrying");
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
            char character = payload[index];
            if (m_sentenceLength < sizeof(sm_szBuffer) - 1)
            {
                sm_szBuffer[m_sentenceLength++] = character;
            }
            else
            {
                m_sentenceLength = 0;
            }

            if (character == '\n')
            {
                sm_szBuffer[m_sentenceLength] = '\0';
                if (!queue_try_add(&m_qSentences, sm_szBuffer))
                {
                    // Should never happen if the queue is sized appropriately. Using the queue_get_max_level()
                    // function (if so compiled) shows the queue never exceeded 1 in testing, so a queue size
                    // of 16 is more than sufficient.
                    printf("Queue full\n");
                }
                m_sentenceLength = 0;
            }
        }
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
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
    bool bFound = false;

    // Check if there are any sentences in the queue. If so, remove one and return it.
    if (queue_try_peek(&m_qSentences, nullptr))
    {
        char szBuffer[GPS_BUFSIZE];
        if (queue_try_remove(&m_qSentences, szBuffer))
        {
            strSentence = std::string(szBuffer);
            bFound      = true;
        }
    }
    return bFound;
}
