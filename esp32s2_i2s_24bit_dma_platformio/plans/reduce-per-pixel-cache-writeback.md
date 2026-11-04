# Plan: Reduce per-pixel `Cache_WriteBack_Addr` calls to one per `fb_flush()`

## Problem

[`mbi_set_pixel()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:904) performs a
`Cache_WriteBack_Addr()` for every pixel (8 DMA entries = 24 bytes). For a 78x78 panel
that is ~6,084 calls per [`fb_flush()`](src/main.cpp:116). Because the ESP32-S2 cache
line is 32 bytes and adjacent pixels share cache lines, the same lines are written back
many times. This is pure overhead when the whole framebuffer is flushed anyway.

## Key facts

- Greyscale region size = `PANEL_SCAN_LINES * PANEL_MBI_LED_CHANS * PANEL_MBI_CHAIN_LEN * 16`
  = 20 * 16 * 5 * 16 = **25,600 entries** (76,800 bytes).
- `mbi_set_pixel()` max touched index = 25,592 + 8 = 25,600, so the region fully covers it.
- `Cache_WriteBack_Addr()` aligns to cache lines internally, so one range call is valid.
- Only [`fb_flush()`](src/main.cpp:116) calls `mbi_set_pixel()`, and it always writes every pixel.

## Changes

### 1. `src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp`
- Declare a new public function:
  `void mbi_cache_writeback();`

### 2. `src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp`
- Add a compile-time constant near the other buffer constants:
  `#define MBI_GRAYSCALE_ENTRIES (PANEL_SCAN_LINES * PANEL_MBI_LED_CHANS * PANEL_MBI_CHAIN_LEN * 16)`
- Remove the per-pixel writeback block inside `mbi_set_pixel()` (the
  `#ifdef MBI_OUTPUT_IN_PSRAM ... Cache_WriteBack_Addr(...) #endif` at the end of the function).
- Add `mbi_cache_writeback()`:
  - Under `#ifdef MBI_OUTPUT_IN_PSRAM`, issue a single
    `Cache_WriteBack_Addr((uint32_t) global_buffer_gclk_cdata, MBI_GRAYSCALE_ENTRIES * sizeof(DMA_DATA_TYPE));`
  - No-op when `MBI_OUTPUT_IN_PSRAM` is not defined.

### 3. `src/main.cpp`
- In [`fb_flush()`](src/main.cpp:116), after the pixel loop and before `mbi_update()`,
  call `mbi_cache_writeback();`.

## Optional follow-ups (not required for this task)
- Collapse the per-entry writeback loop in `mbi_clear()` into one range call over `TOTAL_SIZE`.
- Collapse the init-time per-entry writebacks in `mbi_load_inital_dma_data()`.

## Correctness notes
- The DMA engine reads physical PSRAM directly, so the writeback must happen after all
  pixel writes; placing it at the end of `fb_flush()` satisfies this.
- The existing design already races the continuously looping DMA against buffer writes;
  this change does not make that worse and removes redundant cache-line flushes.
