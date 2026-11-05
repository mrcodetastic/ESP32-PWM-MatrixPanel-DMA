/******************************************************************************************
 * @author      github.com/mrcodetastic
 * @date        2026
 * @brief       ESP32-S3 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/
#pragma once
#include <app_constants.hpp>

// Compile-Time Configuration Constants
#define BYTES_PER_GCLK 3
//#define GCLKS_PER_ROW  513 // clock multiplier off - 6ms per loop
#define GCLKS_PER_ROW  257 // clock multiplier on - 3ms per loop
#define ROWS           20
#define PADDING_BYTES  16

/**
 * 1. Returns the total payload size in bytes using compile-time constants.
 */
size_t get_gclk_payload_total_size(void) {
    int sequence_size = BYTES_PER_GCLK * GCLKS_PER_ROW;
    int block_size = PADDING_BYTES + sequence_size + PADDING_BYTES;
    return (size_t)block_size * ROWS * sizeof(uint8_t);
}

/**
 * 2. Returns the byte value at a specific position within the range.
 */
uint8_t get_gclk_payload_byte_value(int position) {
    int sequence_size = BYTES_PER_GCLK * GCLKS_PER_ROW;
    int block_size = PADDING_BYTES + sequence_size + PADDING_BYTES;
    size_t total_size = get_gclk_payload_total_size();

    // Bounds check
    assert(position >= 0 && position < (int)total_size);

    // Determine row sequence index (0 to ROWS - 1)
    int sequence_index = position / block_size;
    int offset = position % block_size;

    // Calculate position inside main payload sequence
    int sequence_position = offset - PADDING_BYTES;
    int byte_index = sequence_position % BYTES_PER_GCLK;

    // Combine row address bits (row mapped to BIT_A..BIT_E)
    int row = sequence_index & 0x1F;
    uint8_t address_bits = 0;
    if (row & (1 << 0)) address_bits |= BIT_A;
    if (row & (1 << 1)) address_bits |= BIT_B;
    if (row & (1 << 2)) address_bits |= BIT_C;
    if (row & (1 << 3)) address_bits |= BIT_D;
    if (row & (1 << 4)) address_bits |= BIT_E;

    // Determine byte value
    uint8_t byte_value = address_bits;

    // Active payload area (outside padding)
    if (offset >= PADDING_BYTES && offset < PADDING_BYTES + sequence_size) {
        if (byte_index == 2) {
            byte_value |= BIT_GCLK; // 3rd byte in repeat has GCLK pulse high
        }
    }

    return byte_value;
}

// During padding it sets everything to Row 31. Not sure if this did anything.
uint8_t get_gclk_payload_byte_valueV2(int position) {
    int sequence_size = BYTES_PER_GCLK * GCLKS_PER_ROW;
    int block_size = PADDING_BYTES + sequence_size + PADDING_BYTES;
    size_t total_size = get_gclk_payload_total_size();

    // Bounds check
    assert(position >= 0 && position < (int)total_size);

    // Determine row sequence index and offset
    int sequence_index = position / block_size;
    int offset = position % block_size;

    // Check if we are in the padding area (before or after the active sequence)
    bool is_padding = (offset < PADDING_BYTES || offset >= PADDING_BYTES + sequence_size);

    int row;
    if (is_padding) {
        // Option A: Force all address lines HIGH (Row 31 / 0x1F) during padding 
        // to act as a blanking state and drain stray charge/capacitive coupling.
        row = 0x1F; 
    } else {
        // Active payload row (0 to 19)
        row = sequence_index & 0x1F;
    }

    // Combine row address bits (row mapped to BIT_A..BIT_E)
    uint8_t address_bits = 0;
    if (row & (1 << 0)) address_bits |= BIT_A;
    if (row & (1 << 1)) address_bits |= BIT_B;
    if (row & (1 << 2)) address_bits |= BIT_C;
    if (row & (1 << 3)) address_bits |= BIT_D;
    if (row & (1 << 4)) address_bits |= BIT_E;

    // Base byte value includes the address bits (either actual row or padding all-high)
    uint8_t byte_value = address_bits;

    // Active payload area only: handle the GCLK pulse on every 3rd byte
    if (!is_padding) {
        int sequence_position = offset - PADDING_BYTES;
        int byte_index = sequence_position % BYTES_PER_GCLK;
        if (byte_index == 2) {
            byte_value |= BIT_GCLK; // 3rd byte in repeat has GCLK pulse high
        }
    }

    return byte_value;
}


// ------------------ Payload Generation -----------------------------------

DMA_ATTR static uint8_t *spi_tx_octal_payload   = NULL; // data for gclk

esp_err_t allocate_gclk_dma_memory()
{
	/*
	if (spi_tx_octal_payload != NULL) {
          ESP_LOGE("spi_dma_seg_tx_payload", "Looks like we've already called allocate_gclk_dma_memory()!");  		
		  return;
	}
	*/
    
	// We don't want to do this twice
	assert(spi_tx_octal_payload == NULL);

	// Allocate
	size_t alloc_size_bytes  = get_gclk_payload_total_size();
	size_t actual_size = 0;

#ifdef USE_PSRAM
          ESP_LOGI("spi_dma_seg_tx_payload", "Allocating PSRAM DMA memory for global_buffer_gclk_cdata.");  
          esp_err_t err = esp_dma_malloc(alloc_size_bytes, ESP_DMA_MALLOC_FLAG_PSRAM, (void **) &global_buffer_gclk_cdata, &actual_size);
          assert(err == ESP_OK);
#else
          ESP_LOGI("spi_dma_seg_tx_payload", "Allocating SRAM DMA memory for global_buffer_gclk_cdata.");  
          spi_tx_octal_payload = (uint8_t *)(heap_caps_malloc(alloc_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
          assert(spi_tx_octal_payload != NULL);

          actual_size = alloc_size_bytes;
#endif

	ESP_LOGI("spi_dma_seg_tx_payload", "GCLK data total size: %d bytes.", alloc_size_bytes);  

	// Populate based on algo
	for (int i = 0; i < alloc_size_bytes; i++) {
		spi_tx_octal_payload[i] = get_gclk_payload_byte_value(i);
	}

	return ESP_OK;
} // end allocate_gclk_dma_memory