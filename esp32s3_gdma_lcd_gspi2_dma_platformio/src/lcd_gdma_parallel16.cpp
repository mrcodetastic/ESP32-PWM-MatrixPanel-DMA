/*********************************************************************************************
  Simple example of using the ESP32-S3's LCD peripheral for general-purpose
  (non-LCD) parallel data output with DMA. Connect 8 LEDs (or logic analyzer),
  cycles through a pattern among them at about 1 Hz.
  This code is ONLY for the ESP32-S3, NOT the S2, C3 or original ESP32.
  None of this is authoritative canon, just a lot of trial error w/datasheet
  and register poking. Probably more robust ways of doing this still TBD.


 FULL CREDIT goes to AdaFruit and  https://github.com/PaintYourDragon
 
 https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/
 
 https://github.com/adafruit/Adafruit_Protomatter/blob/master/src/arch/esp32-s3.h

 PLEASE SUPPORT THEM!

 ********************************************************************************************/
#if __has_include(<hal/lcd_ll.h>)
// Stop compile errors: /src/platforms/esp32s3/gdma_lcd_parallel16.hpp:64:10: fatal error: hal/lcd_ll.h: No such file or directory

static const char *const TAG = "lcd_gdma_parallel16";

#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#endif

#include "lcd_gdma_parallel16.hpp"
#include "app_constants.hpp" 
#include "esp_attr.h"
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/lldesc.h>


static IRAM_ATTR bool dma_tx_eof_callback(gdma_channel_handle_t chan, gdma_event_data_t *edata, void *user_data) {
  (void)chan;
  (void)edata;
  Bus_Parallel16 *bus = (Bus_Parallel16 *)user_data;
  if (bus) {
    return bus->on_dma_tx_done();
  }
  return false;
}

bool Bus_Parallel16::on_dma_tx_done() {
  BaseType_t high_task_wakeup = pdFALSE;
  if (_transfer_sem) {
    xSemaphoreGiveFromISR(_transfer_sem, &high_task_wakeup);
  }
  return high_task_wakeup == pdTRUE;
}

lcd_cam_dev_t *getDev() {
  return &LCD_CAM;
}

Bus_Parallel16::~Bus_Parallel16() {
  release();
}

int Bus_Parallel16::get_transfer_count() {
  return _trans_done_count;
}

// ------------------------------------------------------------------------------
void Bus_Parallel16::config(const config_t &cfg) {
  _cfg = cfg;
}


//https://github.com/adafruit/Adafruit_Protomatter/blob/master/src/arch/esp32-s3.h
esp_err_t Bus_Parallel16::setup_lcd_dma_periph(void) {
  esp_err_t ret;

  _dev = getDev();

  // LCD_CAM peripheral isn't enabled by default -- MUST begin with this:
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);  // Wait some time for whatever reason


  // Configure PCLK using ESP-IDF's LCD HAL clock calculator and LL setters
  // rather than hand-writing divider register fields. The S3 PLL_F160M source
  // is 160 MHz; ask the HAL for the closest representable rate to 2 MHz.
  constexpr uint32_t LCD_CLOCK_SOURCE_HZ = 160000000;
  constexpr uint32_t LCD_PCLK_TARGET_HZ = 2000000;
  lcd_cam_dev_t *lcd_dev = const_cast<lcd_cam_dev_t *>(_dev);
  lcd_hal_context_t lcd_hal;
  lcd_hal_init(&lcd_hal, 0);
  lcd_ll_select_clk_src(lcd_dev, LCD_CLK_SRC_PLL160M);

  hal_utils_clk_div_t lcd_clk_div = {};
  const uint32_t actual_pclk_hz = lcd_hal_cal_pclk_freq(
      &lcd_hal, LCD_CLOCK_SOURCE_HZ, LCD_PCLK_TARGET_HZ, &lcd_clk_div);
  if (actual_pclk_hz == 0) {
    ESP_LOGE(TAG, "Could not calculate LCD PCLK divider for %u Hz",
             (unsigned)LCD_PCLK_TARGET_HZ);
    return ESP_ERR_INVALID_ARG;
  }

  // Apply the HAL-calculated LCD module divider and run PCLK at the module
  // clock rate (prescale 1). For the 160 MHz source this is approximately
  // 160 MHz / 80 = 2 MHz.
  lcd_ll_set_group_clock_coeff(lcd_dev, lcd_clk_div.integer,
                               lcd_clk_div.denominator, lcd_clk_div.numerator);
  lcd_ll_set_pixel_clock_prescale(lcd_dev, 1);
  lcd_ll_set_pixel_clock_edge(lcd_dev, false);
  lcd_ll_set_clock_idle_level(lcd_dev, false);
  lcd_ll_enable_clock(lcd_dev, true);

  ESP_LOGI(TAG, "LCD PCLK: target=%u Hz, actual=%u Hz, divider=%u+%u/%u",
           (unsigned)LCD_PCLK_TARGET_HZ, (unsigned)actual_pclk_hz,
           (unsigned)lcd_clk_div.integer, (unsigned)lcd_clk_div.numerator,
           (unsigned)lcd_clk_div.denominator);

  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;     // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;  // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;   // Do NOT auto-frame
  LCD_CAM.lcd_misc.lcd_bk_en = 1;           // https://esp32.com/viewtopic.php?t=24459&start=60#p91835


  //  LCD_CAM_DOUTn_MODE (n = 0 ­ 15) The output data bit n is delayed by module clock LCD_CLK.
  //  0: output without delay. 1: delayed by the rising edge of LCD_CLK. 2: delayed by the falling edge
  //  of LCD_CLK. (R/W)
  LCD_CAM.lcd_data_dout_mode.val = 0;  // No data delays

  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // Enable continuous mode for DMA stream
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Valid in 1-byte mode
  LCD_CAM.lcd_user.lcd_byte_order = 0;     // Valid in 2-byte mode (do not invert bytes)
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 1;       // 16-bit data mode

  // No dummy or command phases for shift register / matrix data
  LCD_CAM.lcd_user.lcd_dummy = 1;
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 2; // 5 dummy cycles fo GCLK before stuff is sent to the MBI
  LCD_CAM.lcd_user.lcd_cmd = 0;


  // Route 8 LCD data signals to GPIO pins
  int8_t *pins = _cfg.pin_data;
  gpio_hal_context_t gpio_hal = { GPIO_HAL_GET_HW(0) };

  int routed_pins = 0;
  for (int i = 0; i < 16; i++) {
    if (pins[i] >= 0) {  // -1 value will CRASH the ESP32!
      esp_rom_gpio_connect_out_signal(pins[i], LCD_DATA_OUT0_IDX + i, false, false);
      gpio_hal_func_sel(&gpio_hal, pins[i], PIN_FUNC_GPIO);
      gpio_set_drive_capability((gpio_num_t)pins[i], (gpio_drive_cap_t)3);
      routed_pins++;
    }
  }
  (void)routed_pins;

  // Clock
  esp_rom_gpio_connect_out_signal(_cfg.pin_wr, LCD_PCLK_IDX, _cfg.invert_pclk, false);
  gpio_hal_func_sel(&gpio_hal, _cfg.pin_wr, PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);

  // Remaining descriptor elements are initialized before each DMA transfer.
  // Allocate DMA channel and connect it to the LCD peripheral
  static gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = {
      .reserve_sibling = 0 }
  };
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  // ESP32-S3 uses the AHB GDMA instance (the legacy gdma_new_channel() is deprecated).
  const esp_err_t chan_alloc_ret = gdma_new_ahb_channel(&dma_chan_config, &dma_chan);
#else
  const esp_err_t chan_alloc_ret = gdma_new_channel(&dma_chan_config, &dma_chan);
#endif
  (void)chan_alloc_ret;

  const esp_err_t connect_ret = gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  (void)connect_ret;

  static gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false
  };
  const esp_err_t strategy_ret = gdma_apply_strategy(dma_chan, &strategy_config);
  (void)strategy_ret;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  // Equivalent of the legacy ability { sram_trans_align = 32, psram_trans_align = 64 }:
  // 32-byte burst size, and external (PSRAM) memory access enabled.
  gdma_transfer_config_t transfer_config = {
    .max_data_burst_size = 32,
    .access_ext_mem = true,
  };
  const esp_err_t xfer_cfg_ret = gdma_config_transfer(dma_chan, &transfer_config);
#else
  gdma_transfer_ability_t ability = {
    .sram_trans_align = 32,
    .psram_trans_align = 64,
  };
  const esp_err_t xfer_cfg_ret = gdma_set_transfer_ability(dma_chan, &ability);
#endif
  (void)xfer_cfg_ret;


  if (!_transfer_sem) {
    _transfer_sem = xSemaphoreCreateBinary();
    if (!_transfer_sem) {
      ESP_LOGE(TAG, "DIAG stage 323: FAILED to create transfer semaphore");
      return ESP_ERR_NO_MEM;
    }
  }

  gdma_tx_event_callbacks_t tx_cbs = {
    .on_trans_eof = dma_tx_eof_callback,
  };
  const esp_err_t cb_ret = gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, this);
  (void)cb_ret;

  ret = gdma_reset(dma_chan);  // Reset DMA to known state.
  esp_rom_delay_us(10);

  LCD_CAM.lcd_user.lcd_dout = 1;         // Enable data out
  LCD_CAM.lcd_user.lcd_start = 0;        // Ensure not started yet
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;  // Reset LCD TX FIFO
  LCD_CAM.lcd_user.lcd_update = 1;       // Update registers

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "gdma_reset() failed: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

esp_err_t Bus_Parallel16::release(void) {

  if (dma_chan) {
    const esp_err_t stop_ret = gdma_stop(dma_chan);
    const esp_err_t del_ret = gdma_del_channel(dma_chan);
    (void)stop_ret;
    (void)del_ret;
    dma_chan = nullptr;
  }

  if (_transfer_sem) {
    vSemaphoreDelete(_transfer_sem);
    _transfer_sem = nullptr;
  }

  if (_dmadesc_a) {
    heap_caps_free(_dmadesc_a);
    _dmadesc_a = nullptr;
    _dmadesc_count = 0;
    _dmadesc_count_sram_alloc = 0;
  }

  return ESP_OK;
}

// Need this to work for double buffers etc.
bool Bus_Parallel16::allocate_dma_desc_memory(size_t len) {

  // Only allocate more ram if we don't have enough for the requested number of dma descriptors
  if (len > _dmadesc_count_sram_alloc) {
    HUB75_DMA_DESCRIPTOR_T *new_desc = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(
        sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (new_desc == nullptr) {
      ESP_LOGE(TAG, "DIAG stage 351: ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
      return false;
    }

    if (_dmadesc_a) {
      heap_caps_free(_dmadesc_a);
    }

    _dmadesc_a = new_desc;
    _dmadesc_count_sram_alloc = len;
  }

  // Length
  _dmadesc_count = len;

  return true;
}

esp_err_t Bus_Parallel16::dma_transfer_start() {
  if (!dma_chan || !_dmadesc_a || !_transfer_sem) {
    ESP_LOGE(TAG, "DIAG stage 360: dma_transfer_start() invalid state "
                  "(chan=%p, desc=%p, sem=%p)",
             (void *)dma_chan, (void *)_dmadesc_a, (void *)_transfer_sem);
    return ESP_ERR_INVALID_STATE;
  }

  // Clear any stale semaphore tokens
  xSemaphoreTake(_transfer_sem, 0);

  // Clean starting state for GDMA and LCD async TX FIFO
  LCD_CAM.lcd_user.lcd_start = 0;
  esp_err_t ret = gdma_reset(dma_chan);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DIAG stage 362: gdma_reset() failed: %s", esp_err_to_name(ret));
    return ret;
  }
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  LCD_CAM.lcd_user.lcd_update = 1;
  esp_rom_delay_us(2);

  // Start GDMA first to prefetch descriptors/data into the FIFO
  ret = gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DIAG stage 364: gdma_start() failed: %s", esp_err_to_name(ret));
    return ret;
  }

  esp_rom_delay_us(2);

  // Trigger LCD DMA transfer
  LCD_CAM.lcd_user.lcd_start = 1;

  // Block until GDMA EOF transfer completes
  if (xSemaphoreTake(_transfer_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE(TAG, "DIAG stage 367: Timed out waiting for GDMA TX completion");
    gdma_stop(dma_chan);
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update = 1;
    return ESP_ERR_TIMEOUT;
  }

  // Allow the asynchronous FIFO to fully serialize the final words to the bus pins.
  // At 7 MHz (142 ns/word), the 16-word FIFO takes ~2.3 us to empty. 10 us is safe margin.
  esp_rom_delay_us(10);

  // Stop LCD transfer
  LCD_CAM.lcd_user.lcd_start = 0;
  LCD_CAM.lcd_user.lcd_update = 1;

  // Avoid the deprecated ++/--/compound-assignment operators on volatile types (C++20).
  _trans_done_count = _trans_done_count + 1;

  return ESP_OK;
}  // end

esp_err_t Bus_Parallel16::send_stuff_once(void *data, size_t size_in_bytes, bool is_greyscale_data) {

  (void)is_greyscale_data;

  if (data == nullptr || size_in_bytes == 0) {
    ESP_LOGE(TAG, "DIAG stage 370: send_stuff_once() invalid args (data=%p, size=%u)",
             (void *)data, (unsigned)size_in_bytes);
    return ESP_ERR_INVALID_ARG;
  }

  // Descriptor chunk size must be a multiple of 32 bytes (burst align) and 2 bytes (16-bit word),
  // and <= 4095. 4064 = 127 * 32.
  static constexpr size_t DMA_CHUNK_SIZE = 4064;

  size_t dma_lldesc_required = (size_in_bytes + DMA_CHUNK_SIZE - 1) / DMA_CHUNK_SIZE;
  ESP_LOGV(TAG, "Number of DMA descriptors required for LCD payload is: %u.", (unsigned)dma_lldesc_required);

  // Allocate descriptor block of memory if it hasn't already been allocated
  if (!allocate_dma_desc_memory(dma_lldesc_required)) {
    ESP_LOGE(TAG, "DIAG stage 373: descriptor allocation failed");
    return ESP_ERR_NO_MEM;
  }

  size_t len = size_in_bytes;
  uint8_t *ptr = (uint8_t *)data;
  size_t n = 0;

  while (len > 0) {
    size_t dmachunklen = len;
    if (dmachunklen > DMA_CHUNK_SIZE) {
      dmachunklen = DMA_CHUNK_SIZE;
    }

    _dmadesc_a[n].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    _dmadesc_a[n].dw0.suc_eof = 0;
    _dmadesc_a[n].dw0.size = _dmadesc_a[n].dw0.length = dmachunklen;
    _dmadesc_a[n].buffer = ptr;
    // Only link to the next descriptor if one exists
    _dmadesc_a[n].next = (n + 1 < dma_lldesc_required)
                       ? (dma_descriptor_t *)&_dmadesc_a[n + 1]
                       : nullptr;

    len -= dmachunklen;
    ptr += dmachunklen;

    ESP_LOGV(TAG, "Configured _dmadesc_a at pos %u, memory location %08x, pointing to data of size %u. Next dma desc is %u.",
             (unsigned)n, (uintptr_t)&_dmadesc_a[n], (unsigned)_dmadesc_a[n].dw0.size, (unsigned)(n + 1));
    n++;
  }

  ESP_LOGV(TAG, "Configured _dmadesc_a at pos %u, to have EOF flag = 1.", (unsigned)(n - 1));

  // All done, no looping here this time!
  _dmadesc_a[n - 1].dw0.suc_eof = 1;  // Mark last DMA desc as end of stream.
  _dmadesc_a[n - 1].next = nullptr;   // no next item

  // Send it!
  const esp_err_t start_ret = dma_transfer_start();
  if (start_ret != ESP_OK) {
    ESP_LOGE(TAG, "DIAG stage 376: dma_transfer_start() returned %s (%d)",
             esp_err_to_name(start_ret), (int)start_ret);
  }
  return start_ret;
}


#endif

