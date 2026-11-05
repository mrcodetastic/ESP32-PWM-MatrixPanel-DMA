/******************************************************************************************
 * @file        mbi_dma_interface.h
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       LCD_CAM / GDMA parallel bus + MBI5153 command sequencing.
 *
 * This class owns everything that is "output plumbing" for the DMA LCD side of
 * the panel:
 *
 *   - the ESP32-S3 LCD_CAM parallel bus (via Bus_Parallel16),
 *   - the MBI5153 greyscale DMA frame buffer,
 *   - the scratch buffer used for command / Vsync payloads,
 *   - the MBI5153 command sequences (soft reset, pre-active, config registers,
 *     Vsync) and per-pixel greyscale word packing,
 *   - the DMA_LAT-toggle hardware self-test.
 *
 * It does not own the software RGB framebuffer or drawing primitives. Matrix
 * supplies pixels through setPixel() in the regular build, or passes its
 * pre-converted framebuffer to the optimized renderer, then requests sendFrame().
 *
 * The SPI/GCLK side of the panel (GPSPI2 + address lines) is deliberately NOT
 * part of this class - it is owned by spi_dma_seg_tx_loop.c and driven by
 * Matrix, unchanged.
 ******************************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <esp_err.h>

#include "app_constants.hpp"
#include "lcd_gdma_parallel16.hpp"

#ifdef OPTIMIZE_FOR_SPEED
// Pre-converted 16-bit MBI intensity values used by the direct framebuffer path.
struct Rgb16 {
  uint16_t r;
  uint16_t g;
  uint16_t b;

  Rgb16() : r(0), g(0), b(0) {}
  Rgb16(uint16_t r_, uint16_t g_, uint16_t b_) : r(r_), g(g_), b(b_) {}
};

// Used by Matrix's inline optimized setPixel() implementation.
extern const uint16_t lumConvTab_14bit_mbi[256];
#endif

// ---------------------------------------------------------------------------
// MBI5153 timing / layout constants (previously magic numbers inside Matrix.h).
// These values define the output contract; do not change without re-verifying
// the panel on a logic analyser.
// ---------------------------------------------------------------------------
constexpr int MBI_BITS_PER_CHANNEL         = 16;  // 16-bit greyscale word per channel
constexpr int MBI_COLOUR_BITS              = 8;   // 8-bit colour input, expanded into the 16-bit word
constexpr int MBI_COLOUR_BITS_PER_GROUP    = 3;   // R/G/B per panel quarter
constexpr int MBI_ENTRIES_PER_IC           = PANEL_MBI_CHAIN_LEN * MBI_BITS_PER_CHANNEL;   // 80
constexpr int MBI_ENTRIES_PER_ROW          = PANEL_MBI_LED_CHANS * MBI_ENTRIES_PER_IC;     // 1280

constexpr int MBI_PRE_ACTIVE_LAT_CLOCKS    = 14;   // pre-active = 14 DCLK edges with LE high
constexpr int MBI_SOFT_RESET_LAT_CLOCKS    = 10;   // soft reset = 10 DCLK edges with LE high
constexpr int MBI_VSYNC_LAT_CLOCKS         = 2;
constexpr int MBI_VSYNC_PAYLOAD_LEN        = 200;  // DCLK words used to position the Vsync pulse

constexpr int MBI_CONFIG1_LATCH_TRIGGER    = 4;
constexpr int MBI_CONFIG2_LATCH_TRIGGER    = 8;

constexpr int MBI_CMD_BUFFER_ENTRIES       = 600;  // scratch buffer for command/Vsync payloads

class MbiDmaInterface {
 public:
  MbiDmaInterface() = default;
  ~MbiDmaInterface();

  // Allocate the greyscale / command DMA buffers, configure the LCD_CAM pin
  // map and bring up the LCD_CAM + GDMA peripherals. Call once from setup().
  esp_err_t begin();

  // True once begin() has completed successfully.
  bool ready() const { return _ready; }

  /******************************* MBI command sequences *******************************/

  // Soft reset: 10 DCLK edges with LE high.
  esp_err_t softReset();

  // Pre-activation command: 14 DCLK edges with LE high.
  esp_err_t preActive();

  // Configuration register 1 / 2 writes (shifted into every MBI in the chain).
  esp_err_t sendConfigReg1();
  esp_err_t sendConfigReg2();

  // Full panel bring-up: reset -> pre-active -> config 1 -> reset ->
  // pre-active -> config 2. Each command is sent as its own DMA transfer.
  esp_err_t configurePanel();

  // Vertical sync: positions the LE pulse in the middle of a blank payload so
  // the MBI latches the frame that was just clocked in.
  esp_err_t sendVsync();

  /********************************** Frame data ***************************************/

  // Write one pixel (8-bit RGB) into the MBI greyscale DMA frame buffer.
  // Nothing is sent to the panel until sendFrame() is called.
  void setPixel(uint8_t x, uint8_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data);

#ifdef OPTIMIZE_FOR_SPEED
  // Convert a pre-converted RGB16 framebuffer directly into the greyscale DMA buffer.
  void renderLinearBufferToDMA(const Rgb16 (*pixel_buffer)[PANEL_PHY_RES_X + 2]);
  void renderLinearBufferToDMAv2(const Rgb16 (*pixel_buffer)[PANEL_PHY_RES_X + 2]);  
  
#endif

  // Shift the greyscale DMA frame buffer out over the parallel bus.
  esp_err_t sendFrame();

  // Zero the greyscale DMA frame buffer.
  void clearFrame();

  /********************************* Diagnostics **************************************/

  // Number of completed parallel-bus DMA transfers.
  int getTransferCount() { return _bus.get_transfer_count(); }

  // Direct access to the underlying peripheral wrapper (diagnostics only).
  Bus_Parallel16 &bus() { return _bus; }

 private:
  // Push `words` 16-bit words from the command scratch buffer.
  esp_err_t sendCommand(const char *what, size_t words);
  void clearCommandBuffer();

  Bus_Parallel16 _bus;

  ESP32_GREY_DMA_STORAGE_TYPE *_frame_buffer = nullptr;  // MBI greyscale DMA frame
  ESP32_GREY_DMA_STORAGE_TYPE *_cmd_buffer = nullptr;    // command / Vsync scratch buffer

  size_t _frame_words  = 0;  // greyscale buffer length in words
  size_t _frame_bytes  = 0;  // greyscale buffer length in bytes
  bool _ready = false;
};
