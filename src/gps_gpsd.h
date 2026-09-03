/*
 * GPSD client class
 *
 * (c) 2026 Erik Tkal
 *
 */

#pragma once

#include "gps.h"

#include <queue>

#include "lwip/tcp.h"
#include "pico/sync.h"
#include "pico/util/queue.h"

auto constexpr GPS_BUFSIZE = 256;   // Max NMEA-0183 sentence length is actually 82 characters
auto constexpr GPS_QUEUE_SIZE = 16; // Number of sentences to queue

class GPS_gpsd : public GPS
{
public:
    typedef std::shared_ptr<GPS_gpsd> Shared;

    GPS_gpsd();
    ~GPS_gpsd() override;

    void Initialize() override;

private:
    bool getSentence(std::string& strSentence) override;
    void closeConnection();
    void writeString(const std::string& str);

    static err_t tcpConnected(void* arg, struct tcp_pcb* pcb, err_t err);
    static err_t tcpPoll(void* arg, struct tcp_pcb* pcb);
    static err_t tcpSent(void* arg, struct tcp_pcb* pcb, u16_t len);
    static err_t tcpRecv(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err);
    static void tcpError(void* arg, err_t err);

    err_t onTcpConnected(struct tcp_pcb* pcb, err_t err);
    err_t onTcpPoll(struct tcp_pcb* pcb);
    err_t onTcpSent(struct tcp_pcb* pcb, u16_t len);
    err_t onTcpRecv(struct tcp_pcb* pcb, struct pbuf* p, err_t err);
    void onTcpError(err_t err);

    // TCP RX management
    struct tcp_pcb* m_pTcpPcb {nullptr};
    ip_addr_t m_remoteAddr {};
    char sm_szBuffer[GPS_BUFSIZE] {};
    size_t m_sentenceLength {0};

    // Queue for received sentences
    queue_t m_qSentences;
};
