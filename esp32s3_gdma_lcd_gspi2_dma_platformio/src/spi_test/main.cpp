/******************************************************************************************
 * @file        spi_test/main.cpp
 * @brief       Standalone ESP32-S3 GPSPI2 segmented DMA (GCLK) output test
 *
 * This app exercises ONLY the SPI/GCLK path used by the project
 * (see src/spi_dma_seg_tx_loop.c and include/spi_dma_seg_tx_payload.h).
 *
 * It does NOT touch the LCD peripheral or the MBI matrix logic, so the
 * GPSPI2 octal output can be validated in isolation with a logic analyser.
 *
 * Behaviour:
 *   - Initialises the GPSPI2 host/device and generates the GCLK payload.
 *   - Starts the continuous segmented DMA output loop.
 *   - Runs for 1 second, stops the loop, waits 1 second, then repeats.
 *   - Prints the completed segment transfer count once per cycle.
 *
 * Build/run with the PlatformIO environment:
 *     pio run -e spi_test -t upload
 *     pio device monitor -e spi_test
 ******************************************************************************************/

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "app_constants.hpp"
#include "spi_dma_seg_tx_loop.h"

// Defined in src/spi_dma_seg_tx_loop.c
extern volatile int spi_seg_transfer_count;

// Start the App
void setup(void)
{
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting SPI GCLK test....");

    // The continuous DMA loop can starve the idle task; disable the TWDT.
    esp_task_wdt_deinit();

    // Start GCLK via SPI (also allocates + generates the octal payload)
    esp_err_t err = spi_setup();
    if (err != ESP_OK) {
        Serial.printf("spi_setup() failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    } else {
        Serial.println("spi_setup() complete, GCLK payload ready.");
    }

    
    delay(1000);
    Serial.println("spi_transfer_loop_start()");
    spi_transfer_loop_start(); // start continuous segmented output

    delay(10000);
    Serial.println("spi_transfer_loop_stop()");
    spi_transfer_loop_stop();  // stop at the next segment boundary
}

void loop() {
   
}
