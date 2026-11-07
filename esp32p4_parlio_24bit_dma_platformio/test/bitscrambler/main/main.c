#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/bitscrambler_loopback.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

BITSCRAMBLER_PROGRAM(nibble_swap_program, "nibble_swap");

static const char *TAG = "bitscrambler_test";

static const uint8_t input_data[] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
};

static const uint8_t expected_output[] = {
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
};

static void print_bytes(const char *label, const uint8_t *data, size_t length)
{
    printf("%s", label);
    for (size_t i = 0; i < length; ++i) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

void app_main(void)
{
    uint8_t *output = heap_caps_calloc(1, sizeof(input_data), MALLOC_CAP_DMA);
    if (output == NULL) {
        ESP_LOGE(TAG, "Could not allocate DMA output buffer");
        return;
    }

    const size_t output_capacity = heap_caps_get_allocated_size(output);
    bitscrambler_handle_t bitscrambler;
    ESP_ERROR_CHECK(bitscrambler_loopback_create(
        &bitscrambler, SOC_BITSCRAMBLER_ATTACH_GPSPI2, output_capacity));
    ESP_ERROR_CHECK(bitscrambler_load_program(bitscrambler, nibble_swap_program));

    size_t output_length = 0;
    ESP_ERROR_CHECK(bitscrambler_loopback_run(
        bitscrambler, (void *)input_data, sizeof(input_data), output,
        output_capacity, &output_length));
    bitscrambler_free(bitscrambler);

    print_bytes("input: ", input_data, sizeof(input_data));
    print_bytes("output:", output, output_length);

    const bool passed = output_length == sizeof(expected_output)
        && memcmp(output, expected_output, sizeof(expected_output)) == 0;
    printf("BitScrambler nibble-swap test: %s\n", passed ? "PASS" : "FAIL");
    if (!passed) {
        ESP_LOGE(TAG, "Output did not match the expected nibble-swapped data");
    } else {
        ESP_LOGI(TAG, "Hardware transform produced the expected output");
    }

    heap_caps_free(output);
}