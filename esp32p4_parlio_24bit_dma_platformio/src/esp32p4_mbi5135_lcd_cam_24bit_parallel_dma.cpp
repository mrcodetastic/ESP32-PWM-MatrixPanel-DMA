/******************************************************************************************
 * @file        esp32p4_mbi5135_lcd_cam_24bit_parallel_dma.cpp
 * @brief       ESP32-P4 LCD_CAM + GDMA 24-bit Parallel Driver for MBI5135
 ******************************************************************************************/

#include <sdkconfig.h>
#if !defined(CONFIG_IDF_TARGET_ESP32P4)
  #error "Designed specifically for ESP32-P4 target"
#endif

#include "esp32p4_mbi5135_lcd_cam_24bit_parallel_dma.hpp"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 5)
    #error "ESP-IDF 5.5.5 or newer is required"
#endif

#include <driver/gpio.h>
#include <esp_private/gdma.h>
#include <esp_private/gdma_link.h>
#include <esp_private/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <hal/lcd_ll.h>
#include <hal/gpio_ll.h>

#if __has_include(<soc/gpio_sig_map.h>)
#include <soc/gpio_sig_map.h>
#endif

#if __has_include(<soc/lcd_cam_struct.h>) && __has_include(<soc/lcd_cam_reg.h>)
#include <soc/lcd_cam_struct.h>
#include <soc/lcd_cam_reg.h>
#elif __has_include("soc/lcd_cam_struct.h") && __has_include("soc/lcd_cam_reg.h")
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#else
// Some editor environments do not expose the ESP32-P4 SoC headers.
// The actual ESP-IDF build still provides them, and this guard prevents false
// include errors when the SDK is not fully indexed by the IDE.
#endif

#include <hal/gpio_hal.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <assert.h>
#include <rom/ets_sys.h>
#include "app_constants.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#if defined(MBI_OUTPUT_IN_PSRAM) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
#include "esp_dma_utils.h"
#endif

#include "rom/cache.h"

static const char *TAG = "lcd_cam_24bit";

DMA_DATA_TYPE *global_buffer_gclk_cdata   = NULL;
DMA_DATA_TYPE *global_buffer_configuration      = NULL;

gdma_link_list_handle_t dma_link_gclk_cdata = NULL;
gdma_link_list_handle_t dma_link_configuration = NULL;

static gdma_channel_handle_t dma_chan = NULL;
static volatile bool dma_buffer_sent = false;
static volatile int interrupt_count = 0;

  /*************** Fixed Buffer Serial Bit Pulse Length Values ****************/
// The length in parallel clocks. Actual memory use will be 3 x this.
#define BUFF_BITLEN_VSYNC           (400)    // Half rate data bit length + some clocks for vsync
#define BUFF_BITLEN_CONFIG          (500)   // Config stuff



#define MBI_GRAYSCALE_ENTRIES (PANEL_SCAN_LINES * PANEL_MBI_LED_CHANS * PANEL_MBI_CHAIN_LEN * 16)

const int BYTES_PER_REPEAT  = 3;
const int REPEATS           = 513;
const int SEQUENCE_SIZE     = BYTES_PER_REPEAT * REPEATS;
const int REPEAT_COUNT      = 20;
const int PADDING_SIZE      = 8;
const int TOTAL_SIZE        = (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE) * REPEAT_COUNT;

const uint16_t CIE[256] = {
    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    1,    1,    1,    2,    2,
    2,    2,    2,    2,    2,    2,    2,    3,    3,    3,    3,    3,    3,    3,    3,    4,
    4,    4,    4,    4,    4,    5,    5,    5,    5,    5,    6,    6,    6,    6,    6,    7,
    7,    7,    7,    8,    8,    8,    8,    9,    9,    9,   10,   10,   10,   10,   11,   11,
   11,   12,   12,   12,   13,   13,   13,   14,   14,   15,   15,   15,   16,   16,   17,   17,
   17,   18,   18,   19,   19,   20,   20,   21,   21,   22,   22,   23,   23,   24,   24,   25,
   25,   26,   26,   27,   28,   28,   29,   29,   30,   31,   31,   32,   32,   33,   34,   34,
   35,   36,   37,   37,   38,   39,   39,   40,   41,   42,   43,   43,   44,   45,   46,   47,
   47,   48,   49,   50,   51,   52,   53,   54,   54,   55,   56,   57,   58,   59,   60,   61,
   62,   63,   64,   65,   66,   67,   68,   70,   71,   72,   73,   74,   75,   76,   77,   79,
   80,   81,   82,   83,   85,   86,   87,   88,   90,   91,   92,   94,   95,   96,   98,   99,
  100,  102,  103,  105,  106,  108,  109,  110,  112,  113,  115,  116,  118,  120,  121,  123,
  124,  126,  128,  129,  131,  132,  134,  136,  138,  139,  141,  143,  145,  146,  148,  150,
  152,  154,  155,  157,  159,  161,  163,  165,  167,  169,  171,  173,  175,  177,  179,  181,
  183,  185,  187,  189,  191,  193,  196,  198,  200,  202,  204,  207,  209,  211,  214,  216,
  218,  220,  223,  225,  228,  230,  232,  235,  237,  240,  242,  245,  247,  250,  252,  255,
};

static bool IRAM_ATTR lcd_cam_dma_eof_isr(gdma_channel_handle_t chan, gdma_event_data_t *evt, void *user_data) {
    interrupt_count = interrupt_count + 1;
    dma_buffer_sent = true;
    return false;
}

int get_interrupt_count() {
    return interrupt_count;
}

lcd_cam_dev_t* getDev() {
    return &LCD_CAM;
}

uint8_t getByteValue(int position) {
    int sequenceIndex = position / (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE);
    int offset = position % (PADDING_SIZE + SEQUENCE_SIZE + PADDING_SIZE);
    int sequencePosition = offset - PADDING_SIZE;
    int byteIndex = sequencePosition % BYTES_PER_REPEAT;
    uint8_t repeatValue = sequenceIndex & 0x1F;

    if (offset < PADDING_SIZE || offset >= PADDING_SIZE + SEQUENCE_SIZE) {
        return repeatValue;
    }    
    if (byteIndex == 2) {
        return 0x80 | repeatValue;
    } else {
        return repeatValue;
    }
}

static void _mbl_set_latch_for_config_buff(int latch_length, int &start_pos, bool stay_low = false) {
    for (int i = latch_length; i > 0; i--) {
      global_buffer_configuration[start_pos++].lat = (stay_low) ? 0 : 1;
#ifdef MBI_OUTPUT_IN_PSRAM          
      Cache_WriteBack_Addr((uint32_t) &global_buffer_configuration[start_pos-1], sizeof(DMA_DATA_TYPE));         
#endif          
    }
}

static void _mbi_set_config_dma(int &dma_output_pos, uint16_t config_reg, bool latch, bool reg2) {
    int latch_trigger_point = (reg2) ? 8 : 4;
    for (int bit = 15; bit >= 0; bit--) {
        int bitval = ((config_reg >> bit) & 1);
        if (bitval) {
            global_buffer_configuration[dma_output_pos].b1 = global_buffer_configuration[dma_output_pos].b2 = global_buffer_configuration[dma_output_pos].b3 = global_buffer_configuration[dma_output_pos].b4 = 1;
            global_buffer_configuration[dma_output_pos].r1 = global_buffer_configuration[dma_output_pos].r2 = global_buffer_configuration[dma_output_pos].r3 = global_buffer_configuration[dma_output_pos].r4 = 1;
            global_buffer_configuration[dma_output_pos].g1 = global_buffer_configuration[dma_output_pos].g2 = global_buffer_configuration[dma_output_pos].g3 = global_buffer_configuration[dma_output_pos].g4 = 1;
        }
        if ((bit < latch_trigger_point) && (latch == true)) {
          global_buffer_configuration[dma_output_pos].lat = 1;
        }
#ifdef MBI_OUTPUT_IN_PSRAM          
        Cache_WriteBack_Addr((uint32_t) &global_buffer_configuration[dma_output_pos], sizeof(DMA_DATA_TYPE));         
#endif
        dma_output_pos++;
    }
}

static void _mbi_load_config_reg1(int &buff_start_pos) {
    int ghost_elimination = 0b11;
    int line_num = 20 - 1;
    int gray_scale = 0;
    int gclk_multiplier = 0;
    int current = 15;

    uint16_t config_reg1_val = (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | current;
    for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
      _mbi_set_config_dma(buff_start_pos, config_reg1_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), false);
    }
}

static void _mbi_load_config_reg2(int &buff_start_pos) {
    uint16_t config_reg2_val = 0b1001000000011110;
    for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
      _mbi_set_config_dma(buff_start_pos, config_reg2_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), true);
    }
}

static esp_err_t allocate_dma_link_list(size_t payload_size, void *buffer,
                                        bool loop_back, gdma_link_list_handle_t *link) {
    const uint32_t item_count = (payload_size + DMA_MAX - 1) / DMA_MAX;
    gdma_link_list_config_t link_config = {
        .num_items = item_count,
        .item_alignment = 8,
        .flags = {.check_owner = true},
    };
    esp_err_t err = gdma_new_link_list(&link_config, link);
    if (err != ESP_OK) return err;

    auto *mount_configs = static_cast<gdma_buffer_mount_config_t *>(
        heap_caps_calloc(item_count, sizeof(gdma_buffer_mount_config_t), MALLOC_CAP_8BIT));
    if (mount_configs == nullptr) {
        gdma_del_link_list(*link);
        *link = nullptr;
        return ESP_ERR_NO_MEM;
    }

    size_t remaining = payload_size;
    uint8_t *next_buffer = static_cast<uint8_t *>(buffer);
    for (uint32_t i = 0; i < item_count; ++i) {
        const size_t chunk_size = remaining > DMA_MAX ? DMA_MAX : remaining;
        mount_configs[i].buffer = next_buffer;
        mount_configs[i].buffer_alignment = 4;
        mount_configs[i].length = chunk_size;
        mount_configs[i].flags.mark_eof = (i == item_count - 1);
        mount_configs[i].flags.mark_final = (i != item_count - 1)
            ? GDMA_FINAL_LINK_TO_DEFAULT
            : (loop_back ? GDMA_FINAL_LINK_TO_HEAD : GDMA_FINAL_LINK_TO_NULL);
        remaining -= chunk_size;
        next_buffer += chunk_size;
    }

    err = gdma_link_mount_buffers(*link, 0, mount_configs, item_count, nullptr);
    heap_caps_free(mount_configs);
    if (err != ESP_OK) {
        gdma_del_link_list(*link);
        *link = nullptr;
    }
    return err;
}

esp_err_t i2s_lcd_mode_setup() {
    config_t _cfg;
    _cfg.bus_freq = 6 * 1000 * 1000;
    _cfg.parallel_width = 24;
    _cfg.pin_wr = MBI_DCLK_PIN;
    _cfg.invert_pclk = false;

    // Pin Mapping for 24-bit parallel data output across d0 to d23
    _cfg.pin_d0  = ADDR_A_PIN; 
    _cfg.pin_d1  = ADDR_B_PIN; 
    _cfg.pin_d2  = ADDR_C_PIN; 
    _cfg.pin_d3  = ADDR_D_PIN; 
    _cfg.pin_d4  = ADDR_E_PIN; 
    _cfg.pin_d5  = -1; 
    _cfg.pin_d6  = MBI_LAT_PIN;
    _cfg.pin_d7  = MBI_GCLK_PIN;

    _cfg.pin_d8  = MBI_R1_PIN;  
    _cfg.pin_d9  = MBI_G1_PIN;
    _cfg.pin_d10 = MBI_B1_PIN;
    _cfg.pin_d11 = MBI_R2_PIN;
    _cfg.pin_d12 = MBI_G2_PIN;  
    _cfg.pin_d13 = MBI_B2_PIN; 
    _cfg.pin_d14 = -1;  
    _cfg.pin_d15 = -1; 

    _cfg.pin_d16 = MBI_R3_PIN; 
    _cfg.pin_d17 = MBI_G3_PIN;
    _cfg.pin_d18 = MBI_B3_PIN;
    _cfg.pin_d19 = MBI_R4_PIN;
    _cfg.pin_d20 = MBI_G4_PIN;      
    _cfg.pin_d21 = MBI_B4_PIN;        
    _cfg.pin_d22 = -1;          
    _cfg.pin_d23 = DEBUG_PIN;

    PERIPH_RCC_ACQUIRE_ATOMIC(PERIPH_LCD_CAM_MODULE, lcdcam_ref_count) {
        if (lcdcam_ref_count == 0) {
            lcd_ll_enable_bus_clock(0, true);
            lcd_ll_reset_register(0);
        }
    }
    PERIPH_RCC_ATOMIC() {
        lcd_ll_select_clk_src(0, LCD_CLK_SRC_PLL160M);
        lcd_ll_set_group_clock_coeff(0, 13, 3, 1);
        lcd_ll_enable_clock(&LCD_CAM, true);
    }

    ESP_LOGI(TAG, "Resetting LCD_CAM and configuring controller registers");
    LCD_CAM.lcd_user.lcd_reset = 1;
    esp_rom_delay_us(1000);

    // Configure PLL160M -> 12 MHz LCD clock -> 6 MHz PCLK.
    lcd_ll_set_pixel_clock_prescale(&LCD_CAM, 2);
    lcd_ll_set_pixel_clock_edge(&LCD_CAM, false);
    lcd_ll_set_clock_idle_level(&LCD_CAM, false);

    // Configure LCD controller for generic 24-bit parallel output (i8080 mode)
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode
    LCD_CAM.lcd_rgb_yuv.lcd_conv_enable = 0; 
    LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  
    LCD_CAM.lcd_misc.lcd_bk_en = 1;          
    
    LCD_CAM.lcd_user.lcd_always_out_en = 1;  
    LCD_CAM.lcd_user.lcd_byte_mode = 2;       // 24-bit mode
    lcd_ll_set_data_wire_width(&LCD_CAM, 24);
    LCD_CAM.lcd_user.lcd_dout_bit_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;      
    LCD_CAM.lcd_user.lcd_dummy = 1;          
    LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1; 
    LCD_CAM.lcd_user.lcd_cmd = 0;            

    // Setup GPIO routing via IO MUX / Matrix for 24 data lines and write clock (PCLK)
    ESP_LOGI(TAG, "Routing LCD_CAM data and clock GPIOs");
    for (int i = 0; i < 24; i++) {
        if (_cfg.pin_data[i] >= 0) {
            const int signal = LCD_DATA_OUT_PAD_OUT0_IDX + i;
            ESP_LOGI(TAG, "Routing data bit %d: GPIO %d to signal %d", i, _cfg.pin_data[i], signal);
            esp_rom_gpio_connect_out_signal(_cfg.pin_data[i], signal, false, false);
            ESP_LOGI(TAG, "Connected GPIO matrix for data bit %d", i);
            gpio_ll_func_sel(&GPIO, (uint8_t)_cfg.pin_data[i], PIN_FUNC_GPIO);
            ESP_LOGI(TAG, "Selected GPIO mux for data bit %d", i);
            gpio_set_drive_capability((gpio_num_t)_cfg.pin_data[i], (gpio_drive_cap_t)3);
            ESP_LOGI(TAG, "Routed data bit %d", i);
        }
    }

    if (_cfg.pin_wr >= 0) {
        ESP_LOGI(TAG, "Routing PCLK GPIO %d to signal %d", _cfg.pin_wr, LCD_PCLK_PAD_OUT_IDX);
        esp_rom_gpio_connect_out_signal(_cfg.pin_wr, LCD_PCLK_PAD_OUT_IDX, _cfg.invert_pclk, false);
        gpio_ll_func_sel(&GPIO, (uint8_t)_cfg.pin_wr, PIN_FUNC_GPIO);
        gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);
        ESP_LOGI(TAG, "Routed PCLK");
    }
    ESP_LOGI(TAG, "LCD_CAM GPIO routing complete");

    // LCD_CAM is connected to the AXI GDMA bus on ESP32-P4.
    gdma_channel_alloc_config_t dma_chan_config = {};
    dma_chan_config.direction = GDMA_CHANNEL_DIRECTION_TX;
    ESP_LOGI(TAG, "Allocating AXI GDMA TX channel");
    esp_err_t err = gdma_new_axi_channel(&dma_chan_config, &dma_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate AXI GDMA channel: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "AXI GDMA channel allocated");

    err = gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "GDMA connected to LCD_CAM");

    gdma_tx_event_callbacks_t tx_cbs = {
        .on_trans_eof = lcd_cam_dma_eof_isr
    };
    err = gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "GDMA EOF callback registered");

    gdma_transfer_config_t transfer_config = {
        .max_data_burst_size = 32,
        .access_ext_mem = false
    };
    err = gdma_config_transfer(dma_chan, &transfer_config);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "GDMA transfer configured");

    err = gdma_reset(dma_chan);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(1000);

    LCD_CAM.lcd_user.lcd_dout = 1;
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    ESP_LOGI(TAG, "LCD_CAM and GDMA setup complete");

    return ESP_OK;
}

esp_err_t i2s_dma_buff_allocate() {
    size_t alloc_size_bytes = (TOTAL_SIZE + BUFF_BITLEN_VSYNC) * sizeof(DMA_DATA_TYPE);
    global_buffer_gclk_cdata = static_cast<DMA_DATA_TYPE *>(heap_caps_malloc(alloc_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    assert(global_buffer_gclk_cdata != nullptr);
    memset(global_buffer_gclk_cdata, 0, alloc_size_bytes);

    esp_err_t err = allocate_dma_link_list(alloc_size_bytes, global_buffer_gclk_cdata,
                                           true, &dma_link_gclk_cdata);
    if (err != ESP_OK) return err;

    size_t config_size_bytes = BUFF_BITLEN_CONFIG * sizeof(DMA_DATA_TYPE);
    global_buffer_configuration = static_cast<DMA_DATA_TYPE *>(heap_caps_malloc(config_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    assert(global_buffer_configuration != nullptr);
    memset(global_buffer_configuration, 0, config_size_bytes);

    err = allocate_dma_link_list(config_size_bytes, global_buffer_configuration,
                                 false, &dma_link_configuration);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t mbi_load_inital_dma_data() {
    dma_set_vsync();

    int bitlen = TOTAL_SIZE;
    for (int i = 0; i < bitlen; i++) {
        global_buffer_gclk_cdata[i].byte0 = getByteValue(i);            
    }

    int counter = 0;
    for (int row = 0; row < PANEL_SCAN_LINES; row++) {
      for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) {
        for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) {
         global_buffer_gclk_cdata[counter].lat = 0;
          int latch = (ic == 4) ? 1 : 0;
          int bit_offset = 16;
          while (bit_offset > 0) {
            bit_offset--;  
            if (latch == 1 && bit_offset == 0) {
              global_buffer_gclk_cdata[counter].lat = 1;
            }
            counter++;
          }
        }
      }
    }

    int start_pos = 10;
    _mbl_set_latch_for_config_buff(10, start_pos); 
    _mbl_set_latch_for_config_buff(2, start_pos, true); 
    _mbl_set_latch_for_config_buff(14, start_pos); 
    _mbl_set_latch_for_config_buff(6, start_pos, true); 
    start_pos += 16;                 
    _mbi_load_config_reg1(start_pos);
    _mbl_set_latch_for_config_buff(2, start_pos, true);                  
    _mbl_set_latch_for_config_buff(10, start_pos); 

    return ESP_OK;
}    

esp_err_t mbi_send_config() {
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    gdma_stop(dma_chan);
    gdma_reset(dma_chan);

    dma_buffer_sent = false;
    ESP_ERROR_CHECK(gdma_start(dma_chan, gdma_link_get_head_addr(dma_link_configuration)));
    esp_rom_delay_us(10);
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    LCD_CAM.lcd_user.lcd_start = 1;

    const int64_t timeout_us = 100000;
    const int64_t wait_start_us = esp_timer_get_time();
    while (!dma_buffer_sent && (esp_timer_get_time() - wait_start_us) < timeout_us) {
        vTaskDelay(1);
    }
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    if (!dma_buffer_sent) {
        gdma_stop(dma_chan);
        ESP_LOGE(TAG, "Timed out waiting for MBI configuration DMA completion");
        return ESP_ERR_TIMEOUT;
    }

    gdma_stop(dma_chan);
    return ESP_OK;
}

esp_err_t mbi_start_output_loop() {
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    gdma_stop(dma_chan);
    gdma_reset(dma_chan);

    ESP_ERROR_CHECK(gdma_start(dma_chan, gdma_link_get_head_addr(dma_link_gclk_cdata)));
    esp_rom_delay_us(10);
    LCD_CAM.lcd_user.lcd_update_reg = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
    return ESP_OK;
}

void mbi_clear() {
    for (int i = 0; i < TOTAL_SIZE; i++) {
      global_buffer_gclk_cdata[i].byte1 = 0;
      global_buffer_gclk_cdata[i].byte2 = 0;
    }
}

void dma_set_vsync() {
    int start_pos = TOTAL_SIZE + (BUFF_BITLEN_VSYNC / 2);
    for (int i = 3; i > 0; i--) {
      global_buffer_gclk_cdata[start_pos++].lat = 1;
    }
}

void dma_clr_vsync() {
    int start_pos = TOTAL_SIZE + (BUFF_BITLEN_VSYNC / 2);
    for (int i = 3; i > 0; i--) {
      global_buffer_gclk_cdata[start_pos++].lat = 0;
    }
}

IRAM_ATTR void mbi_set_pixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    if (x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) return;

    r_data = CIE[r_data];
    g_data = CIE[g_data];
    b_data = CIE[b_data];

    x += 2;
    int y_normalised = y % PANEL_SCAN_LINES;
    int bit_start_pos = (1280 * y_normalised) + ((x % 16) * 80) + ((x / 16) * 16);
    int rgb_channel = (y / PANEL_SCAN_LINES);
    uint8_t mask;

    for (int subpixel_colour_bit = 7; subpixel_colour_bit >= 0; --subpixel_colour_bit) {
        mask = 1 << subpixel_colour_bit;
        switch (rgb_channel) {
            case 0:
                global_buffer_gclk_cdata[bit_start_pos].r1 = (mask & r_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].g1 = (mask & g_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].b1 = (mask & b_data) ? 1 : 0;
                break;
            case 1:
                global_buffer_gclk_cdata[bit_start_pos].r2 = (mask & r_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].g2 = (mask & g_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].b2 = (mask & b_data) ? 1 : 0;
                break;
            case 2:
                global_buffer_gclk_cdata[bit_start_pos].r3 = (mask & r_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].g3 = (mask & g_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].b3 = (mask & b_data) ? 1 : 0;
                break;
            case 3:
                global_buffer_gclk_cdata[bit_start_pos].r4 = (mask & r_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].g4 = (mask & g_data) ? 1 : 0;
                global_buffer_gclk_cdata[bit_start_pos].b4 = (mask & b_data) ? 1 : 0;
                break;
        }
        bit_start_pos++;
    }
}

void mbi_cache_writeback() {
    // Internal RAM DMA buffers don't require cache writeback lines like PSRAM.
}

void mbi_update() {}

esp_err_t mbi_start() {
    ESP_LOGI(TAG, "mbi_start() for ESP32-P4 LCD_CAM (IDF %s)", esp_get_idf_version());
    ESP_LOGI(TAG, "Configuring LCD_CAM and GDMA");
    esp_err_t err = i2s_lcd_mode_setup();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "LCD_CAM and GDMA configured");

    ESP_LOGI(TAG, "Allocating DMA buffers");
    err = i2s_dma_buff_allocate();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "DMA buffers allocated");

    ESP_LOGI(TAG, "Preparing initial DMA data");
    err = mbi_load_inital_dma_data();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Initial DMA data prepared");

    ESP_LOGI(TAG, "Sending MBI configuration");
    err = mbi_send_config();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "MBI configuration sent");

    ESP_LOGI(TAG, "Starting continuous DMA output");
    err = mbi_start_output_loop();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Continuous DMA output started");
    return ESP_OK;
}