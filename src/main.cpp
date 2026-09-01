/*
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

// This implementation uses TCPIP to connect to a gpsd server instead of a local GPS module.

#include <iostream>
#include <pico/stdlib.h>
#include "hardware/adc.h"

#if defined(RASPBERRYPI_PICO_W)
#include "pico/cyw43_arch.h"
#else
#error PICO W required
#endif

#include "gps_tft.h"
#include "network_info.h"

#if defined(DISPLAY_PICO_RESTOUCH) // Waveshare Pico-ResTouch-LCD-3.5
#define SPI_DEVICE spi1
#define PIN_DC     8
#define PIN_CS     9
#define PIN_SCK    10
#define PIN_MOSI   11
#define PIN_MISO   12
#define PIN_BL     13
#define PIN_RST    15
#elif defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#define SPI_DEVICE spi_default              // Default is SPI0 for Pico
#define PIN_MISO   PICO_DEFAULT_SPI_RX_PIN  // White  16
#define PIN_CS     PICO_DEFAULT_SPI_CSN_PIN // Org    17
#define PIN_SCK    PICO_DEFAULT_SPI_SCK_PIN // Purple 18
#define PIN_MOSI   PICO_DEFAULT_SPI_TX_PIN  // Blue   19
#define PIN_RST    20                       // Yellow
#define PIN_DC     21                       // Green
#define PIN_BL     22                       // Gray
#elif defined(SEEED_XIAO_RP2040)
// XIAO has spi0 CSn overlap with uart0, so override
#define SPI_DEVICE spi_default              // Default is SPI0 for XIAO
#define PIN_MISO   PICO_DEFAULT_SPI_RX_PIN  // White  4
#define PIN_CS     26                       // Orange
#define PIN_SCK    PICO_DEFAULT_SPI_SCK_PIN // Purple 2
#define PIN_MOSI   PICO_DEFAULT_SPI_TX_PIN  // Blue   3
#define PIN_RST    27                       // Yellow
#define PIN_DC     28                       // Green
#define PIN_BL     29                       // Gray
#elif defined(WAVESHARE_RP2040_ZERO)
// RP2040-Zero has default spi1, which is on the bottom, so use spi0
#define SPI_DEVICE spi0 // override
#define PIN_MISO   4    // White
#define PIN_CS     5    // Org
#define PIN_SCK    6    // Purple
#define PIN_MOSI   7    // Blue
#define PIN_RST    14   // Yellow
#define PIN_DC     15   // Green
#define PIN_BL     29   // Gray
#else
#error unknown board
#endif

// #define USE_WS2812_PIN 12 // Override
// #define USE_LED_PIN 16    // Override

extern "C"
{
    int _getentropy(void* buffer, size_t length)
    {
        (void)buffer;
        (void)length;
        return ENOSYS;
    }
}

int main()
{
    stdio_init_all();
    adc_init();

    // Set up the TFT display
    spi_init(SPI_DEVICE, 80000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_init(PIN_BL);
    gpio_set_dir(PIN_BL, GPIO_OUT);
    gpio_put(PIN_BL, 1);

#if defined(SEEED_XIAO_RP2040)
    // Clear LED(s) on XIAO (default on)
    LED_pico ledBlue(25);  // blue
    LED_pico ledGreen(16); // green
    LED_pico ledRed(17);   // red
#endif

    if (cyw43_arch_init())
    {
        std::cout << "Failed to initialize wifi" << std::endl;
        return 1;
    }

    // Create the LED object
    LED::Shared spLED;
#if defined(USE_WS2812_PIN)
    spLED = std::make_shared<LED_neo>(1, USE_WS2812_PIN);
    spLED->Initialize();
    spLED->SetPixel(0, led_green);
#elif defined(PICO_DEFAULT_WS2812_PIN) && !defined(USE_LED_PIN)
    spLED = std::make_shared<LED_neo>(1, PICO_DEFAULT_WS2812_PIN);
    spLED->Initialize();
    spLED->SetPixel(0, led_green);
#elif defined(USE_LED_PIN)
    spLED = std::make_shared<LED_pico>(USE_LED_PIN);
    spLED->SetIgnore({led_red});
#elif defined(PICO_DEFAULT_LED_PIN)
    spLED = std::make_shared<LED_pico>(PICO_DEFAULT_LED_PIN);
    spLED->SetIgnore({led_red});
#elif defined(RASPBERRYPI_PICO_W)
    spLED = std::make_shared<LED_pico_w>(CYW43_WL_GPIO_LED_PIN);
    spLED->SetIgnore({led_red});
#endif

    // Create the GPS object
    GPS::Shared spGPS = std::make_shared<GPS>();

    // Create the display.  ILI9341 or ILI9488, rotate 270 degrees
#if defined(DISPLAY_ILI948X)
    ILI_TFT::Shared spDisplay = std::make_shared<ILI948X>(SPI_DEVICE, PIN_CS, PIN_DC, PIN_RST, R270DEG);
#elif defined(DISPLAY_ILI934X)
    ILI_TFT::Shared spDisplay = std::make_shared<ILI934X>(SPI_DEVICE, PIN_CS, PIN_DC, PIN_RST, R270DEG);
#else
#error Unsupported display specified
#endif

    // Create the GPS_TFT display object
    GPS_TFT::Shared spDevice = std::make_shared<GPS_TFT>(spDisplay, spGPS, spLED, g_fGmtOffset);

    spDevice->Initialize();

    // Connect to wifi
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM & ~0xf);
    while (true)
    {
        std::cout << "Connecting to wifi..." << std::endl;
        if (cyw43_arch_wifi_connect_timeout_ms(g_szWifiSsid, g_szWifiPassword, CYW43_AUTH_WPA2_AES_PSK, 5000))
        {
            std::cout << "Failed to connect to wifi" << std::endl;
            continue;
        }
        break;
    }
    std::cout << "Connected to " << g_szWifiSsid << std::endl;

    // Run the show
    spDevice->Run();

    cyw43_arch_deinit();

    return 0;
}
