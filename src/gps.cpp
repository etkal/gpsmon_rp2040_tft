/*
 * GPS class
 *
 * Copyright (c) 2024 Erik Tkal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <queue>
#include <iostream>
#include <iomanip>

#include <pico/sync.h>
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "gps.h"
#include "network_info.h"

#define POLL_TIME_SEC 5
#define GPSD_INIT_STRING "?WATCH={\"nmea\":true}\r\n"

static critical_section_t critsec;
static std::queue<std::string> sg_sentenceQueue;

GPS::GPS()
    : m_bExit(false),
      m_bFixTime(false),
      m_bFixPos(false),
      m_bExternalAntenna(true),
      m_bGSVInProgress(false),
      m_pSentenceCallBack(nullptr),
      m_pSentenceCtx(nullptr),
      m_pGpsDataCallback(nullptr),
      m_pGpsDataCtx(nullptr),
      m_pTcpPcb(nullptr)
{
}

GPS::~GPS()
{
}

void GPS::SetSentenceCallback(void* pCtx, sentenceCallback pCB)
{
    m_pSentenceCtx      = pCtx;
    m_pSentenceCallBack = pCB;
}

void GPS::SetGpsDataCallback(void* pCtx, gpsDataCallback pCB)
{
    m_pGpsDataCtx      = pCtx;
    m_pGpsDataCallback = pCB;
}

void GPS::Run()
{
    // Set up connection to gpsd server
    ::ip4addr_aton(g_szGpsdIpAddress, &m_remoteAddr);
    std::cout << "Connecting to " << ip4addr_ntoa(&m_remoteAddr) << " port " << g_nGpsdTcpPort << std::endl;
    m_pTcpPcb = tcp_new_ip_type(IP_GET_TYPE(m_remoteAddr));

    ::tcp_arg(m_pTcpPcb, this);
    ::tcp_poll(m_pTcpPcb, TCP_poll, POLL_TIME_SEC * 2);
    ::tcp_sent(m_pTcpPcb, TCP_sent);
    ::tcp_recv(m_pTcpPcb, TCP_recv);
    ::tcp_err(m_pTcpPcb, TCP_err);
    tcp_nagle_disable(m_pTcpPcb);

    ::cyw43_arch_lwip_begin();
    err_t err = ::tcp_connect(m_pTcpPcb, &m_remoteAddr, g_nGpsdTcpPort, TCP_connected);
    ::cyw43_arch_lwip_end();

    if (err != ERR_OK)
    {
        std::cout << "Error connecting." << std::endl;
    }

    std::string strSentence;
    while (!m_bExit)
    {
        // Read sentence from GPS device
        // tight_loop_contents();
        sleep_ms(1);
        bool bSentenceAvailable = false;
        critical_section_enter_blocking(&critsec);
        if (!sg_sentenceQueue.empty())
        {
            strSentence = sg_sentenceQueue.front();
            sg_sentenceQueue.pop();
            bSentenceAvailable = true;
        }
        critical_section_exit(&critsec);

        if (bSentenceAvailable)
        {
            processSentence(strSentence);
            bSentenceAvailable = false;
        }
    }

    if (nullptr != m_pTcpPcb)
    {
        ::tcp_arg(m_pTcpPcb, nullptr);
        ::tcp_poll(m_pTcpPcb, nullptr, 0);
        ::tcp_sent(m_pTcpPcb, nullptr);
        ::tcp_recv(m_pTcpPcb, nullptr);
        ::tcp_err(m_pTcpPcb, nullptr);

        if (ERR_OK != ::tcp_close(m_pTcpPcb))
        {
            std::cout << "tcp_close failed, calling abort" << std::endl;
            ::tcp_abort(m_pTcpPcb);
        }
        m_pTcpPcb = nullptr;
    }
}

void GPS::processSentence(std::string strSentence)
{
    // Validate the string
    if (!validateSentence(strSentence))
    {
        return;
    }
    std::cout << strSentence.c_str() << std::endl;

    if (NULL != m_pSentenceCallBack)
    {
        (*m_pSentenceCallBack)(m_pSentenceCtx, strSentence);
    }

    if (!m_spGPSData)
    {
        // Guarantee we have an object to update
        m_spGPSData           = std::make_shared<GPSData>();
        m_spGPSData->mSatList = m_mSatListPersistent; // restore previous data
    }

    // Split the string
    std::vector<std::string> elems;
    std::stringstream s_stream(strSentence);
    while (s_stream.good())
    {
        std::string substr;
        getline(s_stream, substr, ','); // get first string delimited by comma
        elems.push_back(substr);
    }

    if (time_us_64() > m_nSatListTime + 30 * 1000 * 1000) // Nothing in 30 seconds, clear vectors
    {
        m_spGPSData->mSatList.clear();
        m_spGPSData->vUsedList.clear();
    }

    if (elems[0] == "$GPGSV")
    {
        // Multipart, clear any previous data and re-gather
        if (elems[2] == "1")
        {
            m_spGPSData->mSatList.clear();
            m_strNumGSV      = elems[1];
            m_bGSVInProgress = true;
        }
        int nNumSatsInGSV = std::min(4, atoi(elems[3].c_str()) - 4 * (atoi(elems[2].c_str()) - 1));
        if (m_bGSVInProgress)
        {
            for (int i = 4; i < 4 + 4 * nNumSatsInGSV; i += 4)
            {
                if (!elems[i].empty() && !elems[i + 1].empty() && !elems[i + 2].empty())
                {
                    uint num  = atoi(elems[i].c_str());
                    uint el   = atoi(elems[i + 1].c_str());
                    uint az   = atoi(elems[i + 2].c_str());
                    uint rssi = elems[i + 3].empty() ? 0 : atoi(elems[i + 3].c_str());
                    m_spGPSData->mSatList.emplace(std::make_pair(num, SatInfo(num, el, az, rssi)));
                }
            }
            if (elems[2] == m_strNumGSV) // Last one received
            {
                m_bGSVInProgress = false;
                m_nSatListTime   = time_us_64();
            }
        }
        return;
    }
    else if (m_bGSVInProgress) // Did not complete
    {
        m_spGPSData->mSatList.clear();
        m_spGPSData->vUsedList.clear();
    }

    if (elems[0] == "$GPRMC")
    {
        if (!elems[1].empty())
        {
            std::string& t          = elems[1];
            m_spGPSData->strGPSTime = t.substr(0, 2) + ":" + t.substr(2, 2) + ":" + t.substr(4, 2) + "Z";
            m_bFixTime              = true;
        }
        else
        {
            m_bFixTime              = false;
            m_spGPSData->strGPSTime = "";
        }
        if (elems[2] == "A")
        {
            if (!elems[3].empty() && !elems[4].empty() && !elems[5].empty() && !elems[6].empty())
            {
                m_bFixPos                 = true;
                m_spGPSData->strLatitude  = convertToDegrees(elems[3], 7) + elems[4];
                m_spGPSData->strLongitude = convertToDegrees(elems[5], 8) + elems[6];
            }
            if (!elems[7].empty())
            {
                double dKnots = std::stod(elems[7].c_str());
                std::stringstream oss;
                if (dKnots < 10.0)
                {
                    oss << std::fixed << std::setfill(' ') << std::setprecision(1) << dKnots << "kn";
                }
                else
                {
                    oss << std::setfill(' ') << std::setprecision(0) << dKnots << "kn";
                }
                m_spGPSData->strSpeedKts = oss.str();
            }
        }
        else
        {
            m_bFixPos = false;
        }
    }

    if (elems[0] == "$GPGGA")
    {
        if (!elems[7].empty())
        {
            m_spGPSData->strNumSats = "Sat: " + elems[7];
        }
        if (!elems[9].empty())
        {
            double dMeters = std::stod(elems[9].c_str());
            std::stringstream oss;
            if (dMeters < 1000.0)
            {
                oss << std::fixed << std::setfill(' ') << std::setprecision(1) << dMeters << "m";
            }
            else
            {
                oss << std::setfill(' ') << std::setprecision(0) << dMeters << "m";
            }
            m_spGPSData->strAltitude = oss.str();
        }
    }

    if (elems[0] == "$GPGSA")
    {
        m_spGPSData->vUsedList.clear();
        m_spGPSData->strMode3D = elems[2] + "D Fix";
        if (elems[2] == "1")
        {
            m_spGPSData->strMode3D = "No Fix";
        }
        for (int i = 3; i < 15; ++i)
        {
            if (!elems[i].empty())
            {
                uint satNum = atoi(elems[i].c_str());
                if (satNum != 0)
                {
                    m_spGPSData->vUsedList.push_back(satNum);
                }
            }
            else
            {
                break;
            }
        }
    }

    if (elems[0] == "$GPRMC")
    {
        if (NULL != m_pGpsDataCallback)
        {
            m_mSatListPersistent = m_spGPSData->mSatList; // Persist the list
            (*m_pGpsDataCallback)(m_pGpsDataCtx, m_spGPSData);
            m_spGPSData.reset();
        }
    }

    if (elems[0] == "$PGTOP") // PA6H
    {
        if (elems[2] == "2")
        {
            m_bExternalAntenna = false;
        }
        if (elems[2] == "3")
        {
            m_bExternalAntenna = true;
        }
    }

    if (elems[0] == "$PCD") // PA1616S
    {
        if (elems[2] == "1")
        {
            m_bExternalAntenna = false;
        }
        if (elems[2] == "2")
        {
            m_bExternalAntenna = true;
        }
    }
}

bool GPS::validateSentence(std::string& strSentence)
{
    // Validate format and remove checksum and CRLF
    size_t nLen = strSentence.size();
    if (nLen < 1 || strSentence[0] != '$')
    {
        return false;
    }
    if (nLen < 6 || strSentence.substr(nLen - 2, 2) != "\r\n" || strSentence[nLen - 5] != '*')
    {
        return false;
    }
    std::string specifiedCheck  = strSentence.substr(nLen - 4, 2);
    std::string calculatedCheck = checkSum(strSentence.substr(1, nLen - 6));
    if (calculatedCheck != specifiedCheck)
    {
        return false;
    }

    strSentence = strSentence.substr(0, nLen - 5);

    return true;
}

std::string GPS::checkSum(const std::string& strSentence)
{
    uint8_t check = 0;
    for (const char& c : strSentence)
    {
        check ^= (uint8_t)c;
    }
    std::stringstream oss;
    oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (unsigned int)check;
    return oss.str();
}

std::string GPS::convertToDegrees(std::string strRaw, int width)
{
    // Convert (D)DDMM.mmmm to decimal degrees
    double dRawAsDouble = stod(strRaw);
    int firstdigits     = int(dRawAsDouble / 100);
    int nexttwodigits   = dRawAsDouble - double(firstdigits * 100);
    double converted    = double(firstdigits) + nexttwodigits / 60.0;
    std::stringstream oss;
    oss << std::fixed << std::setw(width) << std::setfill(' ') << std::setprecision(4) << converted;
    return oss.str();
}

//
// Networking methods

void GPS::tcp_write_string(const std::string& str)
{
    ::cyw43_arch_lwip_begin();
    err_t err = ::tcp_write(m_pTcpPcb, str.c_str(), str.length(), TCP_WRITE_FLAG_COPY);
    ::cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        std::cout << "lwIP tcp_write err " << (int)err << std::endl;
    }
}

// Static public callback methods to set into lwIP

err_t GPS::TCP_connected(void* arg, struct tcp_pcb* pcb, err_t err)
{
    GPS* pGPS = reinterpret_cast<GPS*>(arg);
    return pGPS->tcp_connected(pcb, err);
}

err_t GPS::TCP_poll(void* arg, struct tcp_pcb* pcb)
{
    GPS* pGPS = reinterpret_cast<GPS*>(arg);
    return pGPS->tcp_poll(pcb);
}

err_t GPS::TCP_sent(void* arg, struct tcp_pcb* pcb, u16_t len)
{
    GPS* pGPS = reinterpret_cast<GPS*>(arg);
    return pGPS->tcp_sent(pcb, len);
}

err_t GPS::TCP_recv(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err)
{
    GPS* pGPS = reinterpret_cast<GPS*>(arg);
    return pGPS->tcp_recv(pcb, p, err);
}

void GPS::TCP_err(void* arg, err_t err)
{
    GPS* pGPS = reinterpret_cast<GPS*>(arg);
    pGPS->tcp_err(err);
}

// Private methods to do the real work

err_t GPS::tcp_connected(struct tcp_pcb* pcb, err_t err)
{
    // std::cout << "GPS::tcp_connected err " << (int)err << std::endl;
    if (ERR_OK != err)
    {
        m_bExit = true;
        return ERR_OK;
    }

    // Send the init string to the server
    tcp_write_string(GPSD_INIT_STRING);

    return ERR_OK;
}

err_t GPS::tcp_poll(struct tcp_pcb* pcb)
{
    // std::cout << "GPS::tcp_poll" << std::endl;
    return ERR_OK;
}

err_t GPS::tcp_sent(struct tcp_pcb* pcb, u16_t len)
{
    // std::cout << "GPS::tcp_sent " << len << std::endl;
    return ERR_OK;
}

err_t GPS::tcp_recv(struct tcp_pcb* pcb, struct pbuf* p, err_t err)
{
    static char szSentence[256];
    static uint nRead = 0;

    // Process the buffer(s) received
    cyw43_arch_lwip_check();
    if (p->tot_len > 0)
    {
        // std::cout << "GPS::tcp_recv " << p->tot_len << " err " << (int)err << std::endl;
        for (pbuf* pBuf = p; nullptr != pBuf; pBuf = pBuf->next)
        {
            char* pPayload = reinterpret_cast<char*>(pBuf->payload);
            for (uint i = 0; i < pBuf->len; ++i)
            {
                szSentence[nRead++] = pPayload[i];
                if (pPayload[i] == '\n')
                {
                    szSentence[nRead++]  = '\0';
                    std::string sentence = szSentence;
                    nRead                = 0;
                    critical_section_enter_blocking(&critsec);
                    sg_sentenceQueue.push(sentence);
                    critical_section_exit(&critsec);
                }
            }
        }
        ::tcp_recved(m_pTcpPcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

void GPS::tcp_err(err_t err)
{
    std::cout << "GPS::tcp_err " << err << std::endl;
}
