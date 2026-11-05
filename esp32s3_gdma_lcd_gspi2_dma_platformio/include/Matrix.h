#ifndef MATRIX_H
#define MATRIX_H

#include <driver/gpio.h>
#include "Arduino.h"
#include <esp_log.h>
#include <esp_task_wdt.h>

#include <iostream>


#include "app_constants.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbi_dma_interface.h"
#include "sdkconfig.h"
#include "spi_dma_seg_tx_loop.h"
#include <array>
#include <cstdio>
#include <math.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) \
  {                         \
    int16_t t = a;          \
    a = b;                  \
    b = t;                  \
  }
#endif


static const char *TAG = "Matrix.h";

#ifdef OPTIMIZE_FOR_SPEED

// RGB values are pre-converted MBI5153 intensities in this build.
using RGB = Rgb16;

#else

// Simple 8-bit RGB colour used by the local software framebuffer.
struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  RGB() : r(0), g(0), b(0) {}
  RGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
};

#endif  // OPTIMIZE_FOR_SPEED


/*
  Matrix is the "user facing" layer: a software RGB framebuffer plus drawing
  primitives. It owns no DMA state at all.

  The panel output plumbing is split in two, matching the two hardware buses:

    - MbiDmaInterface (include/mbi_dma_interface.h)
                                    : LCD_CAM parallel bus + GDMA, the MBI5153
                                      greyscale DMA frame and its command
                                      sequences (soft reset / pre-active /
                                      config registers / Vsync).
    - GPSPI2  (spi_dma_seg_tx_loop.c): GCLK and address line generation. Matrix
                                      drives it directly, exactly as before.

  Drawing writes only to framebuffer[][]. update() converts the buffer through
  MbiDmaInterface (or passes it directly in the speed-optimized build), then
  asks the interface to shift the frame out.
*/
class Matrix {

 public:
  Matrix() {  }

  ~Matrix() {
  }

  /***************************************** PANEL INIT  *****************************************/

  esp_err_t start()
  {
    // Bring up the LCD_CAM/GDMA parallel bus and its MBI greyscale buffers.
    const esp_err_t lcd_setup_result = mbi_dma_interface.begin();
    if (lcd_setup_result != ESP_OK) {
      ESP_LOGE(TAG, "LCD_CAM/GDMA setup failed: %s (%d)",
               esp_err_to_name(lcd_setup_result), (int)lcd_setup_result);

      return lcd_setup_result;
    }

    // Setup SPI DMA Output for GCLK and Address Lines
    const esp_err_t spi_setup_result = spi_setup();
    if (spi_setup_result != ESP_OK) {
      ESP_LOGE(TAG, "GPSPI2 setup failed: %s (%d)",
               esp_err_to_name(spi_setup_result), (int)spi_setup_result);
      return spi_setup_result;
    }

    delay (100);

    // Need to have started GCLK before sending the MBI configuration sequence, or the MBI chips will ignore it.
    spi_transfer_loop_start();
    
    // Nor send the MBI configuration sequence to the panel. This is a series of DMA transfers, each of which
    // is a command sequence (soft reset, pre-active, config reg1, soft reset
    mbi_dma_interface.configurePanel();

    _ready = true;
    return ESP_OK;
  }

  /***************************************** FRAME OUTPUT  *****************************************/

  void update() {

    if (!_ready) {
      ESP_LOGE(TAG, "update() called before initMatrix()");
      return;
    }

#if defined(UPDATE_PROFILE)
    const int64_t update_start_us = esp_timer_get_time();
#endif
    flushFramebuffer();
#if defined(UPDATE_PROFILE)
    const int64_t flush_done_us = esp_timer_get_time();
#endif
    mbi_dma_interface.sendFrame();
#if defined(UPDATE_PROFILE)
    const int64_t frame_done_us = esp_timer_get_time();
#endif

    const esp_err_t stop_err = spi_transfer_loop_stop();
#if defined(UPDATE_PROFILE)
    const int64_t stop_done_us = esp_timer_get_time();
#endif
    if (stop_err != ESP_OK) {
      ESP_LOGE(TAG, "GPSPI2 stop failed: %s (%d)", esp_err_to_name(stop_err), (int)stop_err);
    }
    mbi_dma_interface.sendVsync();
#if defined(UPDATE_PROFILE)
    const int64_t vsync_done_us = esp_timer_get_time();
#endif
    esp_err_t start_err = spi_transfer_loop_start();
#if defined(UPDATE_PROFILE)
    const int64_t initial_start_done_us = esp_timer_get_time();
    int64_t restart_done_us = initial_start_done_us;
    int64_t recovery_start_us = initial_start_done_us;
#endif
    if (start_err != ESP_OK) {
      ESP_LOGE(TAG, "GPSPI2 start failed: %s (%d)", esp_err_to_name(start_err), (int)start_err);
      // One recovery attempt: force the engine idle, then re-arm.
#if defined(UPDATE_PROFILE)
      recovery_start_us = esp_timer_get_time();
#endif
      (void)spi_transfer_loop_stop();
      start_err = spi_transfer_loop_start();
#if defined(UPDATE_PROFILE)
      restart_done_us = esp_timer_get_time();
#endif
      if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "GPSPI2 recovery start failed: %s (%d)", esp_err_to_name(start_err), (int)start_err);
      }
    }
#if defined(UPDATE_PROFILE)
    const int64_t update_done_us = esp_timer_get_time();
    ESP_LOGW(TAG, "update() timing: framebuffer flush=%lld us, greyscale DMA frame=%lld us, GPSPI2 stop=%lld us, VSYNC=%lld us, GPSPI2 start=%lld us, recovery=%lld us, total=%lld us",
             (long long)(flush_done_us - update_start_us),
             (long long)(frame_done_us - flush_done_us),
             (long long)(stop_done_us - frame_done_us),
             (long long)(vsync_done_us - stop_done_us),
             (long long)(initial_start_done_us - vsync_done_us),
             (long long)(restart_done_us - recovery_start_us),
             (long long)(update_done_us - update_start_us));

    const double effective_fps = (update_done_us > update_start_us)
                      ? 1000000.0 / (double)(update_done_us - update_start_us)
                      : 0.0;
    ESP_LOGW(TAG, "Max possible frame rate: %.2f frames/sec", effective_fps);
#endif

  }


  /***************************************** DRAWING STUFF  *****************************************/
  void setBrightness(uint8_t newBrightness) { (void)newBrightness; }

  uint8_t getBrightness() const {
    return 255;
  }

  uint8_t getXResolution() {
    return PANEL_PHY_RES_X;
  }

  uint8_t getYResolution() {
    return PANEL_PHY_RES_Y;
  }


#ifdef OPTIMIZE_FOR_SPEED

  // supposedly internal function to set buffer value
  inline void setPixel(int16_t x, int16_t y, RGB colour) {

    if (!_ready) { return; }
    if (x < 0 || y < 0 || x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) { return; }

    // Apply the physical offset directly during storage
    uint8_t target_x = x + 2;

    // Direct memory write: ~5-10 CPU cycles instead of thousands
    framebuffer[y][target_x] = {
      lumConvTab_14bit_mbi[colour.r], // Pre-shifted to bits 15..2 inside the table
      lumConvTab_14bit_mbi[colour.g],
      lumConvTab_14bit_mbi[colour.b]
    };
  }

  // Write a single pixel into the local framebuffer. Nothing is sent to the
  // panel until update() is called.
  void drawPixel(int16_t x, int16_t y, RGB color) {
    setPixel(x, y, color);
  }

  void drawPixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    setPixel(x, y, RGB(r_data, g_data, b_data));
  }

  // includes 565 to 888 conversion
  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    uint8_t r = (color >> 8) & 0xf8;
    uint8_t g = (color >> 3) & 0xfc;
    uint8_t b = (color << 3);
    r |= r >> 5;
    g |= g >> 6;
    b |= b >> 5;

    setPixel(x, y, RGB(r, g, b));
  }

#else

  // supposedly internal function to set buffer value
  inline void setPixel(int16_t x, int16_t y, RGB color) {
    if (!_ready) { return; }
    if (x < 0 || y < 0 || x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) { return; }

    framebuffer[y][x] = color;
  }

  // Write a single pixel into the local framebuffer. Nothing is sent to the
  // panel until update() is called.
  void drawPixel(int16_t x, int16_t y, RGB color) {
    if (x < 0 || y < 0 || x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
      return;
    }

    setPixel(x, y, color);
  }

  void drawPixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    setPixel(x, y, RGB(r_data, g_data, b_data));
  }

  // includes 565 to 888 conversion
  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    uint8_t r = (color >> 8) & 0xf8;
    uint8_t g = (color >> 3) & 0xfc;
    uint8_t b = (color << 3);
    r |= r >> 5;
    g |= g >> 6;
    b |= b >> 5;

    setPixel(x, y, RGB(r, g, b));
  }

#endif


  void drawFastHLine(int16_t x, int16_t y, int16_t w, RGB color) {
    if (y < 0 || y >= PANEL_PHY_RES_Y) {
      return;
    }
    if (x < 0) {
      w += x;
      x = 0;
    }
    if (x + w > PANEL_PHY_RES_X) {
      w = PANEL_PHY_RES_X - x;
    }
    for (int16_t i = 0; i < w; i++) {
      //framebuffer[y][x + i] = color;
      setPixel(x + i, y, color);
    }
  }

  void drawFastVLine(int16_t x, int16_t y, int16_t h, RGB color) {
    if (x < 0 || x >= PANEL_PHY_RES_X) {
      return;
    }
    if (y < 0) {
      h += y;
      y = 0;
    }
    if (y + h > PANEL_PHY_RES_Y) {
      h = PANEL_PHY_RES_Y - y;
    }
    for (int16_t i = 0; i < h; i++) {
      //framebuffer[y + i][x] = color;
      setPixel(x, y + i, color);
    }
  }

  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, RGB color) {
    if (x0 == x1) {
      if (y0 > y1) {
        _swap_int16_t(y0, y1);
      }
      drawFastVLine(x0, y0, y1 - y0 + 1, color);
      return;
    }
    if (y0 == y1) {
      if (x0 > x1) {
        _swap_int16_t(x0, x1);
      }
      drawFastHLine(x0, y0, x1 - x0 + 1, color);
      return;
    }

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
      _swap_int16_t(x0, y0);
      _swap_int16_t(x1, y1);
    }
    if (x0 > x1) {
      _swap_int16_t(x0, x1);
      _swap_int16_t(y0, y1);
    }

    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++) {
      if (steep) {
        drawPixel(y0, x0, color);
      } else {
        drawPixel(x0, y0, color);
      }
      err -= dy;
      if (err < 0) {
        y0 += ystep;
        err += dx;
      }
    }
  }

  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB color) {
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y + h - 1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x + w - 1, y, h, color);
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB color) {
    if (x < 0) {
      w += x;
      x = 0;
    }
    if (y < 0) {
      h += y;
      y = 0;
    }
    if (x + w > PANEL_PHY_RES_X) {
      w = PANEL_PHY_RES_X - x;
    }
    if (y + h > PANEL_PHY_RES_Y) {
      h = PANEL_PHY_RES_Y - y;
    }
    for (int16_t row = 0; row < h; row++) {
      drawFastHLine(x, y + row, w, color);
    }
  }

  void fillScreen(RGB color) {
    fillRect(0, 0, PANEL_PHY_RES_X, PANEL_PHY_RES_Y, color);
  }

  void drawCircle(int16_t x0, int16_t y0, int16_t r, RGB color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    drawPixel(x0, y0 + r, color);
    drawPixel(x0, y0 - r, color);
    drawPixel(x0 + r, y0, color);
    drawPixel(x0 - r, y0, color);

    while (x < y) {
      if (f >= 0) {
        y--;
        ddF_y += 2;
        f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x;

      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 - x, y0 + y, color);
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 - x, y0 - y, color);
      drawPixel(x0 + y, y0 + x, color);
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 + y, y0 - x, color);
      drawPixel(x0 - y, y0 - x, color);
    }
  }

  void fillCircle(int16_t x0, int16_t y0, int16_t r, RGB color) {
    if (r < 0) {
      return;
    }
    int16_t r2 = r * r;
    for (int16_t dy = -r; dy <= r; dy++) {
      int16_t dx = (int16_t)sqrtf((float)(r2 - dy * dy));
      drawFastHLine(x0 - dx, y0 + dy, 2 * dx + 1, color);
    }
  }

  // Push the whole local framebuffer to the panel's greyscale DMA buffer.
  // This only writes into DMA memory owned by MbiDmaInterface; nothing is
  // transmitted until update() calls sendFrame().

  #ifdef OPTIMIZE_FOR_SPEED
  void flushFramebuffer() {
    mbi_dma_interface.renderLinearBufferToDMAv3(framebuffer);
  }
  #else

  void flushFramebuffer() {
    for (int16_t y = 0; y < PANEL_PHY_RES_Y; y++) {
      for (int16_t x = 0; x < PANEL_PHY_RES_X; x++) {
        const RGB &p = framebuffer[y][x];
        mbi_dma_interface.setPixel(x, y, p.r, p.g, p.b);
      }
    }
  }

  #endif

 protected:

  bool _ready = false;

  // Local software framebuffer. Drawing primitives write here; update()
  // flushes the whole buffer to the panel via MbiDmaInterface::setPixel().

#ifdef OPTIMIZE_FOR_SPEED
  RGB framebuffer[PANEL_PHY_RES_Y][PANEL_PHY_RES_X+2] = {};  // Two leading offset pixels are retained for the MBI layout.
#else
  RGB framebuffer[PANEL_PHY_RES_Y][PANEL_PHY_RES_X];
#endif

  /***************************************** PANEL OUTPUT  *****************************************/

  // LCD_CAM parallel bus, GDMA, MBI greyscale frame + command sequences.
  // The SPI/GCLK side is driven directly from this class via the
  // spi_transfer_loop_* / spi_setup() calls above.
  MbiDmaInterface mbi_dma_interface;
};

#endif  // MATRIX_H
