# ESP32-S2 MBI5135 24-bit Parallel LCD/DMA Driver

| Supported Targets | ESP32-S2 |
| ----------------- | -------- |

A bare-metal-ish ESP-IDF/Arduino driver that streams a complete MBI5135 LED-matrix
frame out of the ESP32-S2 **I2S peripheral in LCD mode**, using **24 bits of parallel
GPIO** and **DMA**. The DMA source buffer can live in internal SRAM or in PSRAM.

The reference hardware is the WeMos **ESP32-S2 Mini** (or equivalent clone), driving a
1/20-scan MBI5135 panel with 16 channels per IC and 5 chained ICs per colour.

---

## 1. Overview

The driver does not bit-bang the panel. Instead it builds a single, long, circular DMA
buffer in memory. Every 3-byte entry in that buffer is one **DCLK cycle** of the 24-bit
parallel bus. The I2S peripheral, configured as an LCD controller, walks the buffer
through DMA and toggles the parallel pins for us. The CPU only has to:

1. Write the per-pixel greyscale bits into the buffer (`mbi_set_pixel()`).
2. Flush the CPU cache to PSRAM if the buffer lives there (`mbi_cache_writeback()`).
3. Let the DMA engine loop forever.

Because the buffer is circular and the DMA never stops, the panel is refreshed
continuously. There is no per-frame "start transfer" call.

```
 ┌──────────────────────┐      ┌───────────────────────────┐      ┌──────────────────┐
 │  Software framebuffer│      │  MBI DMA buffer           │      │  ESP32-S2 I2S    │
 │  (main.cpp)          │ ───► │  global_buffer_gclk_cdata │ ───► │  LCD mode + DMA  │ ───► 24 parallel GPIOs
 │  RGB[78][78]         │      │  out24v2[~31,500]         │      │  (circular)      │      (DCLK, LAT, GCLK,
 └──────────────────────┘      └───────────────────────────┘      └──────────────────┘       A-E, R/G/B 1-4)
```

---

## 2. Hardware / pin mapping

Pin definitions live in [`app_constants.h`](src/app_constants.h:20). The default mapping
is for the WeMos S2 Mini:

| Function | GPIO | Notes |
| -------- | ---- | ----- |
| `ADDR_A`..`ADDR_E` | 39, 40, 36, 33, 34 | Row / scan-line address |
| `MBI_GCLK` | 38 | Greyscale clock (the panel's OE pin) |
| `MBI_LAT`  | 37 | Data latch |
| `MBI_DCLK` | 35 | Parallel data clock (I2S WS / pixel clock) |
| `R1/G1/B1` | 18, 21, 16 | First quarter of the panel |
| `R2/G2/B2` | 17, 1, 2 | Second quarter |
| `R3/G3/B3` | 4, 3, 5 | Third quarter |
| `R4/G4/B4` | 7, 6, 8 | Fourth quarter |
| `DEBUG_PIN` | 13 | Spare / scope trigger |

Panel geometry is also in [`app_constants.h`](src/app_constants.h:61):

```c
#define PANEL_SCAN_LINES     20   // 1/20 scan
#define PANEL_MBI_LED_CHANS  16   // channels per MBI IC
#define PANEL_MBI_CHAIN_LEN   5   // chained ICs per colour
#define PANEL_PHY_RES_X      78
#define PANEL_PHY_RES_Y      78
```

---

## 3. The 24-bit parallel word

The DMA buffer element is [`out24v2`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp:46),
a 3-byte struct. Its bit layout is deliberately identical to the GPIO order routed by
the I2S data signals (`I2S0O_DATA_OUT0_IDX` upward):

| Byte | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
| ---- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
| `byte0` | `gclk` | `lat` | – | `e` | `d` | `c` | `b` | `a` |
| `byte1` | – | – | `b2` | `g2` | `r2` | `b1` | `g1` | `r1` |
| `byte2` | – | – | `b4` | `g4` | `r4` | `b3` | `g3` | `r3` |

So one 3-byte entry carries, simultaneously:

* the row address (`a`..`e`),
* the latch (`lat`),
* the greyscale clock (`gclk`),
* 12 greyscale data bits (`r1..r4`, `g1..g4`, `b1..b4`).

The I2S peripheral emits one entry per DCLK cycle, so the DCLK frequency is the rate at
which these 3-byte words are consumed.

> The header comment in
> [`esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp:33)
> shows the byte-to-clock mapping: byte 0 of the struct is DCLK 1, byte 1 is DCLK 2,
> byte 2 is DCLK 3, then the pattern repeats.

---

## 4. The MBI DMA buffer

The main buffer is `global_buffer_gclk_cdata`, an array of `out24v2`. It is built once
at start-up and then looped by DMA forever.

### 4.1 Size

From [`esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:101):

```c
BYTES_PER_REPEAT = 3
REPEATS          = 513          // GCLK pulses per row (MBI5153 default)
SEQUENCE_SIZE    = 3 * 513 = 1539
PADDING_SIZE     = 8
REPEAT_COUNT     = 20           // scan lines
TOTAL_SIZE       = (8 + 1539 + 8) * 20 = 31,100 entries
```

Plus `BUFF_BITLEN_VSYNC = 400` entries appended for the VSYNC pulse, giving
**31,500 entries × 3 bytes ≈ 94.5 KB**.

The greyscale region is the first

```c
PANEL_SCAN_LINES * PANEL_MBI_LED_CHANS * PANEL_MBI_CHAIN_LEN * 16
= 20 * 16 * 5 * 16 = 25,600 entries (76,800 bytes)
```

and the configuration buffer is `BUFF_BITLEN_CONFIG = 500` entries (1,500 bytes).

### 4.2 GCLK / row-address stream (`byte0`)

[`getByteValue()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:116) generates the
`byte0` content on the fly. The buffer is divided into 20 blocks of 1,555 entries. Each
block is:

```
[ 8 padding ][ 513 × (2 low + 1 GCLK-high) ][ 8 padding ]
```

* The row address (`a`..`e`) is the block index (`0..19`) in the low 5 bits.
* The GCLK bit is set on the **third** entry of every 3-entry group, i.e. one GCLK pulse
  every 3 DCLK cycles. This gives the panel time to shift in greyscale data between
  GCLK edges.
* The padding entries keep the row address stable while the panel settles.

### 4.3 Greyscale data (`byte1` / `byte2`)

[`mbi_set_pixel()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:909) writes the
8-bit RGB value (after CIE correction) into the buffer, MSB first, at:

```c
bit_start_pos = (1280 * (y % 20)) + ((x % 16) * 80) + ((x / 16) * 16);
```

* `1280` = 16 channels × 5 chained ICs × 16 bits per row.
* `(x % 16) * 80` selects the channel (80 bits = 5 ICs × 16 bits).
* `(x / 16) * 16` selects the chained IC.
* The 8-bit colour is placed in the top 8 of the 16 greyscale bits; the low bits stay 0
  (14-bit greyscale mode still sends 16 bits, per the MBI5153 datasheet).

The latch is pre-set at start-up in
[`mbi_load_inital_dma_data()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:678):
`lat = 1` on the last bit of the last chained IC (`ic == 4`) for every channel, so each
channel's 80-bit word is latched into the PWM registers.

### 4.4 VSYNC

[`dma_set_vsync()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:879) sets three
consecutive `lat = 1` entries at `TOTAL_SIZE + BUFF_BITLEN_VSYNC/2`. Three LAT pulses in
a row are interpreted by the panel as VSYNC. [`dma_clr_vsync()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:894)
clears them, which effectively freezes the display.

> Because the DMA is asynchronous, changing the VSYNC entries while the engine is
> running can race with the read pointer. The code comments in
> [`dma_set_vsync()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:858) explain
> this hazard.

### 4.5 Configuration / start-up sequence

`global_buffer_configuration` is a separate 500-entry buffer used once at boot by
[`mbi_send_config()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:794). It sends
the MBI5153 reset, pre-active, and configuration-register writes (ghost elimination,
scan-line count, greyscale depth, GCLK multiplier, LED current). See
[`_mbi_load_config_reg1()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:258) and
[`_mbi_load_config_reg2()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:276).

---

## 5. I2S peripheral configuration

All of this happens in
[`i2s_lcd_mode_setup()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:391).

### 5.1 LCD mode

```c
dev->conf2.lcd_en        = 1;   // LCD mode, not normal I2S
dev->sample_rate_conf.tx_bits_mod = 24;  // 24-bit parallel word
dev->conf.tx_dma_equal   = 1;
dev->fifo_conf.tx_fifo_mod = 1;
dev->conf_chan.tx_chan_mod = 0;
```

In LCD mode the I2S **WS** signal becomes the pixel/DCLK clock, and the data is emitted
in parallel on `I2S0O_DATA_OUT0..23`. The GPIO matrix routes those signals to the pins
listed in section 2.

### 5.2 Clock tree and output frequency

The clock registers are set at
[`esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:484`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:484):

```c
dev->clkm_conf.clk_sel      = 2;   // PLL_160M = 160 MHz
dev->clkm_conf.clkm_div_a   = 63;
dev->clkm_conf.clkm_div_b   = 0;   // integer division
dev->clkm_conf.clkm_div_num = 7;   // SRAM build
// or 23 when MBI_OUTPUT_IN_PSRAM is defined
dev->sample_rate_conf.tx_bck_div_num = 2;
```

The clock path is:

```
PLL_160M (160 MHz)
   └─ / clkm_div_num            (7 or 23)
        └─ / tx_bck_div_num     (2)
             └─ LCD-mode WS/pixel clock factor (4)
                  = DCLK
```

The driver's own calculation uses `i2s_parallel_get_memory_width(24) == 4`:

```c
effective_frequency = (160 / clkm_div_num) / 4;
```

| Build | `clkm_div_num` | Approx. DCLK  |
| ----- | -------------- | ------------  |
| Internal SRAM | 7 | **≈ 5.7 MHz** | 
| PSRAM (`MBI_OUTPUT_IN_PSRAM`) | 23 | **≈ 1.7 MHz** |


### 5.3 DMA

* The buffer is described by a linked list of `lldesc_t` descriptors built by
  [`allocate_dma_descriptors_gb()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:322).
* Each descriptor covers at most `DMA_MAX = 4092` bytes.
* The last descriptor links back to the first (`loop_back = true`), so the transfer is
  circular and never stops.
* `dev->lc_conf` enables data and descriptor burst mode.
* `dev->int_ena.out_eof = 1` raises an interrupt at the end of the descriptor chain;
  [`i2s_dma_isr()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:192) just counts
  them and sets `dma_buffer_sent`.

---

## 6. PSRAM cache coherency

When `MBI_OUTPUT_IN_PSRAM` is defined, the DMA buffers are allocated in PSRAM. The CPU
writes go through the cache, but the DMA engine reads physical PSRAM directly. Anything
still sitting dirty in the cache is invisible to DMA, so the panel would send stale data.

The driver therefore calls `Cache_WriteBack_Addr()` after writing:

* the whole buffer at allocation time,
* each entry during the initial GCLK/greyscale/VSYNC setup,
* the whole greyscale region once per frame via
  [`mbi_cache_writeback()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:971).

`mbi_cache_writeback()` is the important one for runtime: it replaces thousands of
per-pixel write-backs with a single range flush over the 76,800-byte greyscale region.
See [`plans/reduce-per-pixel-cache-writeback.md`](plans/reduce-per-pixel-cache-writeback.md)
for the rationale.

---

## 7. Configuration switches

Both switches are independent and are set in [`platformio.ini`](platformio.ini:28):

| Define | Effect |
| ------ | ------ |
| `MBI_OUTPUT_IN_PSRAM` | Allocate the I2S/DMA output buffers in PSRAM. Frees internal SRAM but forces the slower clock divider (`clkm_div_num = 23`). |
| `RGB_FRAMEBUFFER_IN_PSRAM` | Allocate the 78×78×3 software framebuffer in PSRAM at runtime. This is the *drawing* buffer, not the DMA buffer. |

Recommended combination: `MBI_OUTPUT_IN_PSRAM` **on**, `RGB_FRAMEBUFFER_IN_PSRAM`
**off** (keep the small drawing buffer in fast internal SRAM).

If neither is defined, everything lives in internal SRAM and the driver runs at the
faster ~5.7 MHz DCLK.

---

## 8. Public API

Declared in [`esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp:143):

| Function | Purpose |
| -------- | ------- |
| `mbi_start()` | One-shot init: I2S setup, buffer allocation, initial data, config, start DMA loop. |
| `mbi_set_pixel(x, y, r, g, b)` | Write one pixel into the DMA buffer (CIE-corrected). |
| `mbi_cache_writeback()` | Flush the greyscale region to PSRAM (no-op in SRAM builds). Call once per frame. |
| `mbi_clear()` | Zero the greyscale bytes of the whole buffer. |
| `mbi_update()` | Currently a no-op; VSYNC is permanently on. |
| `dma_set_vsync()` / `dma_clr_vsync()` | Enable / disable the VSYNC pulse. |
| `get_interrupt_count()` | Number of DMA EOF interrupts seen (diagnostics). |

Typical use (see [`main.cpp`](src/main.cpp:110)):

```c
mbi_start();

for (int y = 0; y < PANEL_PHY_RES_Y; y++)
    for (int x = 0; x < PANEL_PHY_RES_X; x++)
        mbi_set_pixel(x, y, r, g, b);

mbi_cache_writeback();   // only matters in PSRAM builds
mbi_update();
```

---

## 9. Building and running

Two PlatformIO environments are provided:

```ini
[env:lolin_s2_mini]        ; official espressif32 platform
[env:lolin_s2_mini_idf555] ; pioarduino fork (ESP-IDF 5.5.x)
```

The source guards ESP-IDF version-specific register fields, so the same tree builds
against both. ESP-IDF **5.1 – 5.x** is required; 6.x is rejected at compile time.

```bash
pio run -e lolin_s2_mini
pio run -e lolin_s2_mini -t upload
pio device monitor --port COMx --dtr 1 --rts 1
```

`--dtr 1 --rts 1` is needed on the S2 Mini because the Arduino USB-CDC object only
transmits once the host asserts the control lines.

---

## 10. Limitations and notes

* **PSRAM throughput** is the hard ceiling. The DMA engine reading PSRAM cannot sustain
  the same DCLK as internal SRAM; the divider is deliberately raised to 23.
* **Asynchronous buffer updates.** The DMA loops continuously while the CPU writes new
  pixels. There is no double-buffering or tear-free sync; a frame can be partially
  updated while it is being scanned out. This is acceptable for the demo but is the
  main source of visual tearing.
* **VSYNC races.** Toggling VSYNC while DMA is running can produce a partial pulse; see
  the comment block above [`dma_set_vsync()`](src/esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.cpp:858).
* **GCLK vs greyscale alignment.** The GCLK/row-address stream is 1,555 entries per row
  while the greyscale data is 1,280 entries per row. They share the same DCLK stream but
  are generated independently; the panel relies on LAT and its internal scan counter
  rather than on a one-to-one alignment of the two regions.
* **ESP32-S2 only.** The register map and GPIO signal indices are S2-specific.
