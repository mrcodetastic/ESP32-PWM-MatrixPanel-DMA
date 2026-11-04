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

// Simple 8-bit RGB colour used by the local software framebuffer.
struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  RGB() : r(0), g(0), b(0) {}
  RGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
};

/*
  // MBI5152 Application Note V1.00- EN

  Section 2: The Setting of Gray Scale
  The setting of gray scale data describes as below.
  1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M
  (M≦16)
  2. The data sequence of cascaded IC is ICnICn-1…. IC2IC1.
  3. The data sequence of each channel is ch15ch14…. ch0.
  4. The data length of each channel is 16-bits, and the default PWM mode is 16-bits. The sequence of gray
  sacle is bit15bit14bit13…bit0 as figure 4 shows. The 14-bits gray scale can be set through Bit[7]=1
  in configuration register 1, and the sequence of gray scale data is bit13bit12… bit0 0 0, the
  last 2-bits (LSB) are set to “0”.
  5. The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data.
  6. LE executes the data latch to send gray scale data into SRAM. Each 16xN bits data needs a “data latch
  command”, where N means the number of cascaded driver.
  7. After the last data latch, it needs at least 50 GCLKs to read the gray scale data into internal display buffer
  before the Vsync command comes.
  8. Display is updated immediately when MBI5152 receives the Vsync signal.
  9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signal.
  10. The period of dead time (ie. The 1025th GCLK) must be larger than 100ns.

  The gray scale data needs the GCLK to save the data into SRAM. The frequency of GCLK must be higher
  than 20% of DCLK to get the correct data.

  // MBI5153
  After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display
  buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives
  the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.

  The DMA framing for the above lives in MbiDmaInterface (include/mbi_dma_interface.h).
*/

static const char *TAG = "Matrix.h";

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

  Drawing writes only to framebuffer[][]; update() flushes it into
  MbiDmaInterface via setPixel() and then asks it to shift the frame out.
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

#if defined(MATRIX_PROFILE)
    const int64_t profile_start_us = esp_timer_get_time();
#endif
    flushFramebuffer();
#if defined(MATRIX_PROFILE)
    const int64_t flush_done_us = esp_timer_get_time();
#endif
    mbi_dma_interface.sendFrame();
#if defined(MATRIX_PROFILE)
    const int64_t frame_done_us = esp_timer_get_time();
#endif

    const esp_err_t stop_err = spi_transfer_loop_stop();
    if (stop_err != ESP_OK) {
      ESP_LOGE(TAG, "GPSPI2 stop failed: %s (%d)", esp_err_to_name(stop_err), (int)stop_err);
    }
    mbi_dma_interface.sendVsync();
     esp_err_t start_err = spi_transfer_loop_start();
    if (start_err != ESP_OK) {
      ESP_LOGE(TAG, "GPSPI2 start failed: %s (%d)", esp_err_to_name(start_err), (int)start_err);
      // One recovery attempt: force the engine idle, then re-arm.
      (void)spi_transfer_loop_stop();
      start_err = spi_transfer_loop_start();
      if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "GPSPI2 recovery start failed: %s (%d)", esp_err_to_name(start_err), (int)start_err);
      }
    }
#if defined(MATRIX_PROFILE)
    const int64_t update_done_us = esp_timer_get_time();
    ESP_LOGI(TAG, "profile update: flush=%lld us, greyscale DMA=%lld us, VSYNC/GCLK control=%lld us, total=%lld us",
             (long long)(flush_done_us - profile_start_us),
             (long long)(frame_done_us - flush_done_us),
             (long long)(update_done_us - frame_done_us),
             (long long)(update_done_us - profile_start_us));
#endif

    //log_e("tsfr count: %d", mbi_dma_interface.getTransferCount());

    //mbi_dma_interface.clearFrame();
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

  // Write a single pixel into the local framebuffer. Nothing is sent to the
  // panel until update() is called.
  void drawPixel(int16_t x, int16_t y, RGB color) {
    if (x < 0 || y < 0 || x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
      return;
    }
    framebuffer[y][x] = color;
  }

  void drawPixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    drawPixel(x, y, RGB(r_data, g_data, b_data));
  }

  // includes 565 to 888 conversion
  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    uint8_t r = (color >> 8) & 0xf8;
    uint8_t g = (color >> 3) & 0xfc;
    uint8_t b = (color << 3);
    r |= r >> 5;
    g |= g >> 6;
    b |= b >> 5;

    drawPixel(x, y, RGB(r, g, b));
  }

  void writePixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    drawPixel(x, y, r_data, g_data, b_data);
  }

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
      framebuffer[y][x + i] = color;
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
      framebuffer[y + i][x] = color;
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
  void flushFramebuffer() {
    for (int16_t y = 0; y < PANEL_PHY_RES_Y; y++) {
      for (int16_t x = 0; x < PANEL_PHY_RES_X; x++) {
        const RGB &p = framebuffer[y][x];
        mbi_dma_interface.setPixel(x, y, p.r, p.g, p.b);
      }
    }
  }

 protected:

  bool _ready = false;

  // Local software framebuffer. Drawing primitives write here; update()
  // flushes the whole buffer to the panel via MbiDmaInterface::setPixel().
  RGB framebuffer[PANEL_PHY_RES_Y][PANEL_PHY_RES_X];

  /***************************************** PANEL OUTPUT  *****************************************/

  // LCD_CAM parallel bus, GDMA, MBI greyscale frame + command sequences.
  // The SPI/GCLK side is driven directly from this class via the
  // spi_transfer_loop_* / spi_setup() calls above.
  MbiDmaInterface mbi_dma_interface;
};

#endif  // MATRIX_H
