# ESP32-S3 MBI5153 SPWM LED Matrix

A hardware-driven LED matrix experiment for the **ESP32-S3** and cascaded **MBI5153 grayscale PWM LED drivers**. The project combines a familiar RGB drawing API with DMA-fed panel signals: the CPU prepares image data, while the ESP32-S3 peripherals do the repetitive, timing-sensitive shifting and clocking.

> **Target software:** ESP-IDF **v5.5.5**. The PlatformIO environments select the Arduino framework, which is built on ESP-IDF; this is not a standalone ESP-IDF application. The low-level SPI/GDMA code checks the IDF version and is specifically developed against 5.5.5. Newer IDF releases may change the private driver structures this project uses.

## Why this is an interesting approach

The panel needs two different high-rate signal streams, and the project assigns each to a suitable ESP32-S3 peripheral:

1. **Grayscale pixel data:** the ESP32-S3 **LCD_CAM** peripheral is repurposed as a general-purpose parallel shift engine. **GDMA** feeds it packed 16-bit words, driving the panel's RGB data outputs, DCLK, and latch (LAT) without bit-banging each clock in software.
2. **GCLK and row addressing:** **GPSPI2** runs in octal mode with a DMA descriptor ring and segmented transfers. Its data outputs provide the grayscale clock (GCLK) and row-address signals as a prebuilt repeating stream. The continuous loop avoids asking the CPU to regenerate scan timing for every row and frame.

This is the clever part: the ESP32-S3's LCD and SPI hardware are used as deterministic signal generators, not just as interfaces to conventional LCD/SPI devices. The CPU remains free to build the next image while DMA/peripheral hardware handles the large volume of precisely timed panel traffic. It also avoids a CPU-driven GPIO edge for every pixel bit.

The design keeps drawing, MBI protocol framing, and scan-signal generation in separate layers. That makes it easier to reason about panel commands and data packing without mixing them into the drawing primitives. The panel is updated deliberately: pixel data is shifted, GCLK is stopped for the latch/Vsync boundary, the new frame is latched, and the repeating GCLK/address stream is restarted.

## Hardware and panel layout

- Target MCU: **ESP32-S3** (the LCD_CAM and GPSPI2 implementation is S3-specific).
- LED driver: cascaded **MBI5153** devices using grayscale PWM.
- Configured scan layout: 20 scan lines, 16 channels per driver, and five chained drivers per channel.
- Configured logical panel area: **78 × 78 pixels**. The MBI packing layout reserves an **80 × 80** address space and applies a two-pixel horizontal offset for the missing/unused positions.
- The project provides compile-time GPIO maps for the S3 dev board and the LiveGrid PCB in [app_constants.hpp](include/app_constants.hpp). Select the matching PlatformIO environment and verify the actual board wiring before connecting a panel.

The GPIO maps, signal polarity, panel scan order, chain length, MBI configuration words, and clock rates are hardware-specific. They are not universal HUB75 pinouts. Confirm the panel datasheet and wiring, and use a logic analyzer when adapting the project to another panel or PCB.

## How a frame reaches the LEDs

1. [Matrix drawing operations](include/Matrix.h:188) update a software RGB framebuffer; drawing does not immediately transmit pixels.
2. The framebuffer is converted to the MBI DMA layout. Each 8-bit RGB component is mapped through a CIE 1931 luminance lookup table, then expanded into the 16-bit channel word format. Pixel coordinates are rearranged for the scan-line, channel, and cascaded-driver order expected by the panel.
3. LCD_CAM/GDMA shifts the packed grayscale data over the parallel bus. LAT is inserted at the end of the appropriate channel/driver data groups.
4. GPSPI2's repeating DMA stream supplies GCLK and row-address timing. The current SPI setup configures an approximately **8 MHz** GCLK stream; the LCD_CAM data clock is configured for approximately **2 MHz**. These are implementation values to validate against the actual panel and wiring.
5. At the frame boundary, the GPSPI2 loop is stopped, a separate LCD_CAM DMA payload emits the MBI Vsync/LAT pulse, and the GPSPI2 stream is restarted. Dedicated command scratch memory prevents command transfers from overwriting the active grayscale frame buffer.

The MBI bring-up sequence includes soft reset, pre-activation, configuration-register writes, and Vsync. The frame format and command timing are documented alongside the implementation because small changes to LAT placement or clock counts can produce a blank or corrupted panel.

## Project map

- [src/main.cpp](src/main.cpp) — Arduino [setup()](src/main.cpp:16)/[loop()](src/main.cpp:53) and the animated RGB demo.
- [include/Matrix.h](include/Matrix.h) — drawing API, software framebuffer, and overall start/update sequencing.
- [include/mbi_dma_interface.h](include/mbi_dma_interface.h) and [src/mbi_dma_interface.cpp](src/mbi_dma_interface.cpp) — MBI5153 command sequencing, pixel packing, luminance conversion, and LCD_CAM DMA frame output.
- [include/lcd_gdma_parallel16.hpp](include/lcd_gdma_parallel16.hpp) and [src/lcd_gdma_parallel16.cpp](src/lcd_gdma_parallel16.cpp) — LCD_CAM/GDMA parallel-bus implementation.
- [include/spi_dma_seg_tx_loop.h](include/spi_dma_seg_tx_loop.h) and [src/spi_dma_seg_tx_loop.c](src/spi_dma_seg_tx_loop.c) — GPSPI2 segmented DMA loop for GCLK and address signals. This code uses ESP-IDF private SPI/GDMA interfaces and is the most version-sensitive part of the project.
- [include/app_constants.hpp](include/app_constants.hpp) — pin maps, panel geometry, and MBI configuration constants.
- [src/lcd_test](src/lcd_test) and [src/spi_test](src/spi_test) — standalone peripheral test applications, selectable through their PlatformIO environments.
- [tools/esp32_s3_parallel_spi_dma_payload_visualizer_logic_analyzer.html](tools/esp32_s3_parallel_spi_dma_payload_visualizer_logic_analyzer.html) — browser-based visualizer for inspecting the parallel SPI DMA payload.

## Build, upload, and monitor

Install PlatformIO and use the environment matching the board and pin map. The PlatformIO configuration targets Espressif32 platform `~8.0.0`, Arduino framework, and ESP-IDF 5.5.5.

| Environment | Intended target | GPIO map |
| --- | --- | --- |
| `esp32-s3-devkitc-1-n8r2-livegrid` | LiveGrid PCB, 8 MB flash / 2 MB PSRAM | LiveGrid |
| `yd-esp32-s3-n16r8-china` | YD ESP32-S3 board, 16 MB flash / 8 MB PSRAM | S3 dev board |
| `esp32-s3-devkitc1-n8r8` | ESP32-S3 dev board, 8 MB flash / 8 MB PSRAM | S3 dev board |

From the project directory, for example:

```sh
pio run -e esp32-s3-devkitc-1-n8r2-livegrid
pio run -e esp32-s3-devkitc-1-n8r2-livegrid -t upload
pio device monitor -e esp32-s3-devkitc-1-n8r2-livegrid
```

Substitute the environment name that matches your hardware. The default application is the animated matrix demo. The separate peripheral tests can be built and uploaded with `pio run -e lcd_test -t upload` or `pio run -e spi_test -t upload`. Serial monitoring is configured for **115200 baud**.

### ESP-IDF version note

The SPI segmented-transfer implementation requires ESP-IDF **5.5.5 or newer** at compile time and emits a warning when built with a version newer than 5.5.5. That warning matters: the implementation reaches into private ESP-IDF SPI/GDMA internals, which are not a stable public API. For reproducible builds, use the configured PlatformIO platform and confirm its installed Arduino-ESP32 package is based on ESP-IDF 5.5.5. A successful compile on a newer version does not by itself guarantee correct runtime behavior.

## Adapting the project

1. Choose or define the correct GPIO mapping in [app_constants.hpp](include/app_constants.hpp) and make the corresponding PlatformIO build flag select it.
2. Confirm panel scan ratio, dimensions, cascaded-driver count, RGB lane ordering, clock polarity/rate, and MBI5153 configuration against the exact panel.
3. Start with the `lcd_test` and `spi_test` environments and inspect DCLK, GCLK, LAT, and address lines with a logic analyzer before running the full demo.
4. Keep the MBI payload lengths, latch placement, and configured GCLK count consistent. The MBI configuration controls the expected clocks per row; changing that configuration requires updating and verifying the generated scan stream too.

This project is an experimental, hardware-specific driver rather than a drop-in library for arbitrary matrix panels. Treat its low-level peripheral setup and panel timings as a coordinated system.
