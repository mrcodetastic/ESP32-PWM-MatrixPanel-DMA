# `matrix.update()` Performance Assessment

## Bottom line

Based on the timing sample preserved in the source comments, the largest part of
`matrix.update()` is framebuffer conversion, not descriptor setup or the 16-bit
DMA send:

| `update()` phase | Sample elapsed time | At 240 MHz* | Assessment |
|---|---:|---:|---|
| Framebuffer flush / `renderLinearBufferToDMAv3` | 32.8 ms | 7.9 million cycles | Largest cost; mostly CPU work |
| Greyscale LCD DMA send | 6.5 ms | 1.6 million cycles elapsed | Mostly transfer/wait time, not CPU computation |
| Stop continuous GPSPI2 output | 4.3 ms | 1.0 million cycles elapsed | Waits for the current SPI segment boundary |
| VSYNC command | 13.3 ms | 3.2 million cycles elapsed | Surprisingly high; logging may be a major contributor |
| Restart GPSPI2 output | 0.05 ms | 12,500 cycles | Small |
| **Total** | **57.0 ms** | **13.7 million cycles elapsed** | About **17.6 updates/sec** |

*The repository does not specify the running CPU frequency, so the cycle column
is illustrative, assuming 240 MHz. These are elapsed-time equivalents, not
measurements of CPU cycles spent executing instructions. DMA and semaphore
waits contribute elapsed time without keeping the CPU busy.

## Call chain and likely costs

### 1. Framebuffer conversion

`Matrix::update()` calls `flushFramebuffer()`, which in the optimized build calls
`renderLinearBufferToDMAv3()`. The optimized renderer visits 20 scan rows by 80
output columns, transposes a 16-lane bit matrix for each position, then writes
16 output samples. That is about 1,600 transposes and 25,600 DMA-buffer writes
per frame. This is the dominant measured phase.

- [`update()`](include/Matrix.h)
- [`renderLinearBufferToDMAv3()`](src/mbi_dma_interface.cpp)

### 2. Greyscale frame DMA

`sendFrame()` calls `send_stuff_once()`, which calculates the descriptor count,
fills a descriptor chain, then calls `dma_transfer_start()`. The frame is
51,328 bytes, split into 13 descriptors. The count calculation is one
division/round-up; descriptor setup fills only 13 entries. Descriptor memory
is reused after its initial allocation.

The much larger 6.5 ms measurement is the DMA transfer and wait for completion,
not the descriptor arithmetic.

- [`sendFrame()`](src/mbi_dma_interface.cpp)
- [`send_stuff_once()`](src/lcd_gdma_parallel16.cpp)
- [`dma_transfer_start()`](src/lcd_gdma_parallel16.cpp)

### 3. Stop continuous GPSPI2 output

`spi_transfer_loop_stop()` requests that the continuous GPSPI2 stream stop at a
segment boundary and polls until the peripheral is idle. The GCLK payload is
16,060 bytes, and the configured SPI clock is 4 MHz in the current source; in
octal mode, a segment is roughly 4 ms. That is close to the recorded 4.3 ms.
This is primarily synchronization/waiting, not expensive computation.

- [`spi_transfer_loop_stop()`](src/spi_dma_seg_tx_loop.c)
- [`spi_wait_usr_idle()`](src/spi_dma_seg_tx_loop.c)

### 4. VSYNC command

`sendVsync()` clears the 600-word command buffer, sets the two LAT words, appends
trailing clocks, scans 200 command words, and sends 216 words. That is little
data and little computation; 13.3 ms is unexpectedly long for the payload.
`sendCommand()` emits an `ESP_LOGI` line. If INFO logging is enabled and output
goes over a slow serial connection, that log could account for a significant
part of the measured time. Confirm by comparing with INFO logging disabled.

- [`sendVsync()` and `sendCommand()`](src/mbi_dma_interface.cpp)

### 5. Restart GPSPI2 output

`spi_transfer_loop_start()` resets a small SPI descriptor ring, resets/enables
the GDMA channel and FIFO, and re-arms the peripheral. The recorded 52 µs is
small. A second stop/start happens only after a start failure.

- [`spi_transfer_loop_start()`](src/spi_dma_seg_tx_loop.c)

## Measurement caveats

The values above come from timing samples preserved in source comments, not a
fresh measurement of the current build. In the current `platformio.ini`,
`MATRIX_PROFILE` is enabled but `UPDATE_PROFILE` is commented out. The per-phase
timings in `update()` are guarded by `UPDATE_PROFILE`, so those figures will not
be produced by that configuration unless it is enabled.

Also, `update()` captures its total before printing its profiling logs, but its
caller's `render+update` interval includes any logs printed inside `update()`.
For a clean comparison, avoid per-frame logging while collecting the outer FPS
too.

## Assessment

Prioritize the renderer, then investigate the VSYNC phase and its logging.
Precomputing DMA descriptor counts or descriptor fields is unlikely to make a
meaningful difference.
