/******************************************************************************************
 * @file        spi_dma_seg_tx_loop.c
 * @author      github.com/mrcodetastic
 * @date        2024
 * @updated     2026
 * @brief       ESP32-S3 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/

/***************************************************************************************
   Abusing the ESP32S3's GPSPI2 for Continuous / Endless Transmit (TX) Loop of 
   Octal SPI (OSPI) output using configure-segmented transfer of < 32kB each segment.

   When a segment loop is completed an interrupt will be triggered.

   This enables us not just to be limited to the LCD device with the DMA loop output of
   data to whatever peripheral we want to have connected to continuously receive a
   stream of information.

   Code modified from: https://github.com/MCJack123/craftos-esp/tree/master/main/driver
   Credit to: https://github.com/MCJack123/

   Updated for ESP-IDF 5.5.5 by github.com/mrcodetastic

 ****************************************************************************************/

#include "esp_idf_version.h"

// --- ESP-IDF Version Lock ---
#define REQUIRED_IDF_MAJOR 5
#define REQUIRED_IDF_MINOR 5
#define REQUIRED_IDF_PATCH 5

#define REQUIRED_IDF_VERSION_VAL ESP_IDF_VERSION_VAL(REQUIRED_IDF_MAJOR, REQUIRED_IDF_MINOR, REQUIRED_IDF_PATCH)

#if ESP_IDF_VERSION < REQUIRED_IDF_VERSION_VAL
  #error "Compilation failed: This driver requires ESP-IDF v5.5.5 or higher due to internal GDMA/SPI driver structure changes."
#elif ESP_IDF_VERSION > REQUIRED_IDF_VERSION_VAL
  #warning "Compilation warning: A newer ESP-IDF version than v5.5.5 was detected. Internal SPI/GDMA driver fields may have changed."
#endif

#include <Arduino.h>
#include <sdkconfig.h>

// CONFIG_IDF_TARGET_ESP32S3 is defined by sdkconfig.h, so this check must
// come after that include. Use GCC's #warning (not MSVC's #pragma warning).
#if !defined CONFIG_IDF_TARGET_ESP32S3
#warning "Not an ESP32S3! What are you doing?"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_private/spi_common_internal.h>
#include "esp_private/gdma.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <hal/spi_hal.h>
#include <hal/spi_ll.h>
#include <hal/gdma_ll.h>
#include <soc/soc.h>
#include <soc/periph_defs.h>
#include <soc/lldesc.h>
#include <soc/gdma_struct.h>
#include <soc/io_mux_reg.h>

// Custom includes
#include "spi_dma_seg_tx_loop.h"
#include "spi_dma_seg_tx_payload.h"
#include "app_constants.hpp"

static const char *TAG = "gpspi2_spi_dma_tx_loop";

/**************************************************************************************
 * Notes on using SPI & DMA in a continuous loop:
 * 1) Any single SPI transmission must be less than 32kB!
 * 2) DMA descriptors point to ~4KB payloads (~8 linked descriptors for 32kB).
 * 3) Total DMA payload length must match the CONF state transmission length.
 *
 **************************************************************************************/

DMA_ATTR volatile int  spi_seg_transfer_count = 0;

esp_err_t spi_transfer_initial_payload(void);
esp_err_t spi_dma_seg_setup(void);
static bool spi_wait_usr_idle(uint32_t timeout_us);

// volatile: the CPU writes the SPI_USR_CONF_NXT bit while the DMA engine reads
// this same word every lap. Without volatile the compiler may reorder the
// store relative to the GDMA start.
DMA_ATTR static volatile uint32_t spi_seg_conf_1[4];
DMA_ATTR static uint32_t spi_seg_conf_value_nxt_true;
DMA_ATTR static uint32_t spi_seg_conf_value_nxt_false;

static int dma_lldesc_required = 0; 
DMA_ATTR static lldesc_t *dma_data_lldesc = NULL; 

/**************************************************************************************/
static spi_device_handle_t spi_device;

/**************************************************************************************/
#define spi_dma_ll_tx_reset(dev, chan)  gdma_ll_tx_reset_channel(&GDMA, chan);
#define spi_dma_ll_tx_start(dev, chan, addr) do { \
    gdma_ll_tx_set_desc_addr(&GDMA, chan, (uint32_t)addr); \
    gdma_ll_tx_start(&GDMA, chan); \
  } while (0)

// Timeout for waiting on the SPI engine to reach a known state. Must be
// declared before spi_transfer_loop_start()/stop() use it.
#define SPI_SEG_STOP_TIMEOUT_US  50000   // tune to >= 2 segment periods

/**************************************************************************************/

uint32_t spi_get_transfer_count(void) {
  return (uint32_t)spi_seg_transfer_count;
}

uint32_t get_gpspi2_intr_val(void){
  return GPSPI2.dma_int_st.val;
}

/**************************************************************************************/
// Step 1 - Create individual dma descriptors for payload
int lldesc_setup_chunk(lldesc_t *dmadesc, const void *data, int len, int offset) {
    int n = offset;
    const uint8_t *src = (const uint8_t *)data;
    while (len) {
        int dmachunklen = len;
        if (dmachunklen > LLDESC_MAX_NUM_PER_DESC) {
            dmachunklen = LLDESC_MAX_NUM_PER_DESC;
        }
        dmadesc[n].size = dmachunklen;
        dmadesc[n].length = dmachunklen;
        dmadesc[n].buf = (uint8_t *)src;
        dmadesc[n].eof = 0;
        dmadesc[n].sosf = 0;
        dmadesc[n].owner = 1;
        dmadesc[n].qe.stqe_next = NULL;
        len -= dmachunklen;
        src += dmachunklen;
        n++;
    }
    return n;
}

// Step 2 - Link dma link list descriptors
void lldesc_setup_chain(lldesc_t *dmadesc, int dma_desc_count, bool loop) {
  dma_desc_count--;
  // A continuous circular ring must NOT mark the tail as EOF: the GDMA would
  // otherwise raise an EOF event on every lap. Only a one-shot chain ends
  // with eof = 1.
  dmadesc[dma_desc_count].eof = loop ? 0 : 1;

  if (loop == true) {
    dmadesc[dma_desc_count].qe.stqe_next = &dmadesc[0];
  }
    
  int n = dma_desc_count;
  while (n > 0) {
    dmadesc[n-1].eof = 0;    
    dmadesc[n-1].qe.stqe_next = &dmadesc[n];     
    n--;
  }
  ESP_LOGI(TAG, "lldesc_setup_chain() completed.");
}

/**************************************************************************************/
esp_err_t spi_setup(void)
{
  esp_err_t err = ESP_OK;

  spi_bus_config_t host_conf = { 0 };

  // From: https://github.com/MCJack123/craftos-esp/blob/master/main/driver/vga.c
  //
  // The DMA driver can only send 32k at a time, but we need to send 185k. How fix?
  // The concept is to use multiple chained DMA transfers, using the CONF
  // buffer feature to split up the large data buffer into multiple transfers.
  // This is because the maximum size of any single transfer is 32kB (256kb),
  // so by splitting it up into multiple transfers which trigger each other,
  // we can achieve larger and faster transfers.
  // To save on CPU time, we also make the transfer loop back to itself, which
  // makes the framebuffer essentially act as an infinite transfer.
  // Also, to trigger Vsync, we'll have the first and last transfers in the
  // loop trigger an interrupt, which will deassert and assert the Vsync line,
  // respectively.

  // To set this up without rewriting the entire driver and HAL, we'll abuse
  // some internals of the driver to get the raw data we need without having
  // to generate it all manually. We'll start by creating linked lists for the
  // non-first chunks. We'll then initiate a transfer for the first chunk, but
  // in the pre-transfer callback, we'll link the rest of the generated chunks
  // onto the end of the generated transfer list. We'll also set SPI_USER_CONF
  // to enable configure-segmented transfers. After that, we should be good to
  // let it run.

  // ensure GND is connected on the logic analyser when testing!
  host_conf.data0_io_num = SPI_OCTAL_UNUSED1;
  host_conf.data1_io_num = SPI_OCTAL_UNUSED2;
  host_conf.sclk_io_num  = -1; // clock not used  (therefore don't have SPICOMMON_BUSFLAG_SCLK below )

  host_conf.data2_io_num = ADDR_A_PIN;
  host_conf.data3_io_num = ADDR_B_PIN; 

  // If any of data4_io_num to data7_io_num do not have GPIO outputs assigned (in octal mode) 
  // then there will be a "Guru Meditation Error: Core  1 panic'ed (LoadProhibited). Exception was unhandled."
  host_conf.data4_io_num = ADDR_C_PIN;
  host_conf.data5_io_num = ADDR_D_PIN; 
  host_conf.data6_io_num = ADDR_E_PIN;
  host_conf.data7_io_num = MBI_GCLK;//ADDR_A_PIN

  host_conf.max_transfer_sz = 32768; //32768 is the max for S3
  host_conf.flags = SPICOMMON_BUSFLAG_OCTAL | SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_MASTER;
    host_conf.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

  // 1. Initialize SPI bus (Handles default internal interrupt allocation cleanly)
  ESP_LOGD(TAG, "Initializing SPI bus");    
  CHECK_CALLE(spi_bus_initialize(SPI2_HOST, &host_conf, SPI_DMA_CH_AUTO), "Could not initialize SPI bus");

  // --- ADDED: Enforce internal pull-downs on custom signal/address pins ---
  gpio_set_pull_mode(MBI_GCLK, GPIO_PULLDOWN_ONLY);
  gpio_set_pull_mode(ADDR_A_PIN, GPIO_PULLDOWN_ONLY);
  gpio_set_pull_mode(ADDR_B_PIN, GPIO_PULLDOWN_ONLY);
  gpio_set_pull_mode(ADDR_C_PIN, GPIO_PULLDOWN_ONLY);
  gpio_set_pull_mode(ADDR_D_PIN, GPIO_PULLDOWN_ONLY);  
  gpio_set_pull_mode(ADDR_E_PIN, GPIO_PULLDOWN_ONLY);    
  // Repeat for any other active data/address pins you use
  

  // 2. Initialize device
  ESP_LOGD(TAG, "Initializing SPI device");
  spi_device_interface_config_t device_conf = { 0 };
  device_conf.command_bits   = 0;
  device_conf.address_bits   = 0;
  device_conf.dummy_bits     = 0;
  device_conf.mode           = 0;
  device_conf.clock_source   = SPI_CLK_SRC_DEFAULT;
  device_conf.clock_speed_hz = 500000; // 500Khz. This is the initial speed for the first transfer. The actual speed will be set by the DMA loop. In the spi_dma_seg_setup() function, the GPSPI2.clock registers are configured for 8MHz operation.
  device_conf.spics_io_num   = -1; 
  device_conf.flags          = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
  device_conf.queue_size     = 1;
  device_conf.pre_cb         = NULL;
  device_conf.post_cb        = NULL;
  
  CHECK_CALLE(spi_bus_add_device(SPI2_HOST, &device_conf, &spi_device), "Could not initialize SPI device");

  ESP_LOGD(TAG, "spi_setup() complete");  

  // Populate the generated GCLK payload before it is referenced by the
  // initial transfer and the segmented DMA setup.
  CHECK_CALLE(allocate_gclk_dma_memory(), "Could not allocate GCLK DMA payload");

  spi_transfer_initial_payload();
  spi_dma_seg_setup();

  return ESP_OK;
}

// Configure registers via test payload.
esp_err_t spi_transfer_initial_payload(void)
{
  esp_err_t err;

  ESP_LOGD(TAG, "Sending Initial Transfer: spi_transfer_setup()");

  static spi_transaction_t trans;
  trans.flags     = SPI_TRANS_MODE_OCT;
  trans.tx_buffer = spi_tx_octal_payload; 
  //trans.override_freq_hz = SPI_MASTER_FREQ_8M;
  trans.length    = (get_gclk_payload_total_size()) * 8;
  trans.rx_buffer = NULL; 
  trans.rxlength  = 0;

  CHECK_CALLE(spi_device_polling_start(spi_device, &trans, portMAX_DELAY), "Could not start SPI transfer");
  // polling_start() only acquires the bus and kicks the transfer off; it does
  // NOT wait for completion. Without polling_end() the transfer is still
  // shifting when spi_dma_seg_setup() reprograms GPSPI2.clock and when the
  // first spi_transfer_loop_start() resets GDMA. Block here so the peripheral
  // is genuinely idle (cmd.usr == 0) before the loop is armed.
  CHECK_CALLE(spi_device_polling_end(spi_device, portMAX_DELAY), "Could not end SPI transfer");

  return ESP_OK;
}

// Setup DMA segments and config.
esp_err_t spi_dma_seg_setup(void)
{
  esp_err_t err = ESP_OK;

  ESP_LOGD(TAG, "Setup SPI DMA-controlled configurable segmented transfer");

  assert(dma_lldesc_required == 0);

  // --- CRITICAL FIX: Configure GP-SPI2 clock registers AFTER initial payload ---
  // This ensures the ESP-IDF driver doesn't overwrite your dividers during polling start.   

  
  // 8Mhz
  /*
  GPSPI2.clock.clkdiv_pre = 0;
  GPSPI2.clock.clkcnt_n = 9;   // Divisor = 10 -> 80MHz / 10 = 8 MHz (assuming 80MHz APB)
  GPSPI2.clock.clkcnt_l = 9;
  GPSPI2.clock.clkcnt_h = 4;
  GPSPI2.clock.clk_equ_sysclk = 0;  
  */

  // 1Mhz
/*  
  GPSPI2.clock.clkdiv_pre = 7;     // Pre-divider = 7 + 1 = 8
  GPSPI2.clock.clkcnt_n = 9;       // Main divider = 9 + 1 = 10 (Total divisor = 8 * 10 = 80)
  GPSPI2.clock.clkcnt_l = 9;       // Must be equal to clkcnt_n in master mode
  GPSPI2.clock.clkcnt_h = 4;       // floor((clkcnt_n + 1) / 2 - 1) = floor(10 / 2 - 1) = 4
  GPSPI2.clock.clk_equ_sysclk = 0; // Use divided system clock
*/

// 2Mhz
GPSPI2.clock.clkdiv_pre = 1;     // Pre-divider = 1 + 1 = 2
GPSPI2.clock.clkcnt_n = 10;      // Main divider = 10 + 1 = 11 (Total divisor = 2 * 11 = 22)
GPSPI2.clock.clkcnt_l = 10;      // Must be equal to clkcnt_n in master mode
GPSPI2.clock.clkcnt_h = 4;       // floor((10 + 1) / 2 - 1) = 4
GPSPI2.clock.clk_equ_sysclk = 0; // Use divided system clock

/*
  // 2.96 MHz
  GPSPI2.clock.clkdiv_pre = 2;     // Pre-divider = 2 + 1 = 3
  GPSPI2.clock.clkcnt_n = 8;       // Main divider = 8 + 1 = 9 (Total divisor = 3 * 9 = 27)
  GPSPI2.clock.clkcnt_l = 8;       // Must match clkcnt_n in master mode
  GPSPI2.clock.clkcnt_h = 3;       // floor((8 + 1) / 2 - 1) = floor(9 / 2 - 1) = 3
  GPSPI2.clock.clk_equ_sysclk = 0; // Use divided system clock  
*/
  spi_seg_conf_1[0]  = 0xA0000000UL;          // Magic Value
  spi_seg_conf_1[0] |= (1 << 3) | (1 << 6);  // SPI_USER_REG | SPI_MS_DLEN_REG
  //spi_seg_conf_1[0] |= (1 << 4); // Include SPI_CLOCK_REG in segment updates

  spi_seg_conf_value_nxt_true   = GPSPI2.user.val | SPI_USR_CONF_NXT;
  spi_seg_conf_value_nxt_false  = GPSPI2.user.val & ~(SPI_USR_CONF_NXT); 

  // If this bit is set, it means this configurable segmented transfer will continue its next transaction (segment).
  spi_seg_conf_1[1]  = spi_seg_conf_value_nxt_true; 

  // Must match exactly the dma payload total chunk size -1
  spi_seg_conf_1[2]  = (get_gclk_payload_total_size() * 8) -1; 

  dma_lldesc_required = 1; 
  dma_lldesc_required += lldesc_get_required_num(get_gclk_payload_total_size()); 

  dma_data_lldesc = (lldesc_t*) heap_caps_malloc(sizeof(lldesc_t) * dma_lldesc_required, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);  

  int offset = 0;
  offset = lldesc_setup_chunk(dma_data_lldesc, (const void *)spi_seg_conf_1, 4*3, 0);
  offset = lldesc_setup_chunk(dma_data_lldesc, spi_tx_octal_payload, get_gclk_payload_total_size(), offset); 

  lldesc_setup_chain(dma_data_lldesc, dma_lldesc_required, true); 

  return err;
}

esp_err_t spi_transfer_loop_start(void)
{
  esp_err_t err = ESP_OK;

  ESP_LOGD(TAG, "Starting Output Loop");

  // 0. The peripheral must be fully halted before we touch GDMA or re-arm.
  //    Writing cmd.usr = 1 while it is already 1 is silently ignored, which
  //    is the classic "GCLK dies after a while" failure. If the previous
  //    stop() lost its race, fail loudly instead of pretending to start.
  if (!spi_wait_usr_idle(SPI_SEG_STOP_TIMEOUT_US)) {
    ESP_LOGE(TAG, "start(): SPI peripheral did not go idle, aborting");
    return ESP_ERR_TIMEOUT;
  }

  spi_seg_conf_1[1] = spi_seg_conf_value_nxt_true;
  __sync_synchronize();
  GPSPI2.slave.dma_seg_magic_value = 0xA;

  // --- FIX 1: Reset descriptor ownership and flags for restarting ---
  if (dma_data_lldesc != NULL) {
      for (int i = 0; i < dma_lldesc_required; i++) {
          dma_data_lldesc[i].owner = 1;
      }
      // Circular ring: the tail must not be EOF (see lldesc_setup_chain()).
      dma_data_lldesc[dma_lldesc_required - 1].eof = 0;
      dma_data_lldesc[dma_lldesc_required - 1].qe.stqe_next = &dma_data_lldesc[0];
  }

  const spi_dma_ctx_t *dma_ctx = spi_bus_get_dma_ctx(SPI2_HOST);
  int tx_chan_id = -1;
  esp_err_t chan_err = gdma_get_channel_id(dma_ctx->tx_dma_chan, &tx_chan_id);
  if (chan_err != ESP_OK || tx_chan_id < 0) {
    ESP_LOGE(TAG, "gdma_get_channel_id failed: %s (%d)", esp_err_to_name(chan_err), (int)chan_err);
    return (chan_err != ESP_OK) ? chan_err : ESP_ERR_INVALID_STATE;
  }

  // Rewind the DMA ring to the head (start of the payload). This is a stream
  // reset, not a full peripheral re-init: the SPI is already idle above.
  spi_dma_ll_tx_reset(&GDMA, tx_chan_id);
  spi_ll_dma_tx_fifo_reset(&GPSPI2);
  spi_ll_outfifo_empty_clr(&GPSPI2);
  spi_ll_dma_tx_enable(&GPSPI2, 1);
  spi_dma_ll_tx_start(&GDMA, tx_chan_id, &dma_data_lldesc[0]);

  GPSPI2.slave.usr_conf = 1;
  GPSPI2.misc.cs0_dis = 1;

  GPSPI2.dma_int_clr.dma_seg_trans_done = 1;
  GPSPI2.dma_int_clr.trans_done = 1;
  GPSPI2.dma_int_clr.seg_magic_err = 1;

  GPSPI2.cmd.conf_bitlen = 0;
  GPSPI2.user.cs_setup = 0;
  GPSPI2.user1.cs_setup_time = 0;
  GPSPI2.user1.cs_hold_time = 0;
  GPSPI2.user1.usr_addr_bitlen = 0;
   
  GPSPI2.cmd.update = 1;
  while (GPSPI2.cmd.update);

  GPSPI2.cmd.usr = 1;

  // Verify the re-arm actually latched; otherwise the stream is dead.
  if (GPSPI2.cmd.usr == 0) {
    ESP_LOGE(TAG, "start(): cmd.usr did not latch, engine not running");
    return ESP_ERR_INVALID_STATE;
  }

  // Liveness counter: increments once per successful (re)start.
  spi_seg_transfer_count++;

  return err;
}

/* If you set dma_int_ena.dma_seg_trans_done = 1, the line asserts, and the ISR 
 * that runs is the SPI master driver's (spicommon_irqhandler) because it owns ETS_SPI2_INTR_SOURCE. 
 * That ISR knows nothing about dma_seg_trans_done and does not clear it, so the level-triggered line stays 
 * asserted and the ISR re-enters immediately, forever. 
 * 
 * Your polling loop then never gets to run the disable — on a single core it is a hard hang / WDT reset. 
 * That's the trap; the polling itself is not the problem.
*/

// Waits until the SPI peripheral has actually halted (cmd.usr == 0).
// dma_seg_trans_done is asserted at the end of *every* segment, so it cannot
// prove the engine stopped; cmd.usr is the real "idle" condition.
static bool spi_wait_usr_idle(uint32_t timeout_us)
{
  while (GPSPI2.cmd.usr != 0) {
    if (GPSPI2.dma_int_raw.seg_magic_err) {
      GPSPI2.dma_int_clr.seg_magic_err = 1;
      ESP_LOGE(TAG, "seg_magic_err while waiting for USR idle");
      return false;
    }
    if (timeout_us == 0) {
      ESP_LOGE(TAG, "USR idle timeout, cmd=%08lx raw=%08lx",
               (unsigned long)GPSPI2.cmd.val,
               (unsigned long)GPSPI2.dma_int_raw.val);
      return false;
    }
    esp_rom_delay_us(1);
    timeout_us--;
  }
  return true;
}

// --- FIX 2: Poll hardware status register directly instead of relying on a custom ISR ---
esp_err_t spi_transfer_loop_stop(void)
{
  ESP_LOGD(TAG, "Calling spi_transfer_loop_stop()");

  // 1. Baseline: drop any completion left over from the running loop
  GPSPI2.dma_int_clr.dma_seg_trans_done = 1;

  // 2. Ask the segmented engine not to continue past the next boundary
  spi_seg_conf_1[1] = spi_seg_conf_value_nxt_false;
  __sync_synchronize();

  // 3. Wait until the peripheral has actually halted. dma_seg_trans_done is
  //    set at every segment boundary, so it cannot prove the engine stopped;
  //    cmd.usr == 0 is the real "idle" condition.
  bool ok = spi_wait_usr_idle(SPI_SEG_STOP_TIMEOUT_US);

  // 4. Clear it so the next start() sees a clean flag
  GPSPI2.dma_int_clr.dma_seg_trans_done = 1;

  return ok ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t spi_dma_transfer_loop_unpause(void) 
{
  esp_err_t err = ESP_OK;;

  ESP_LOGD(TAG, "Calling spi_dma_transfer_loop_unpause()");    

  dma_data_lldesc[dma_lldesc_required-1].eof = 0;
  dma_data_lldesc[dma_lldesc_required-1].qe.stqe_next = &dma_data_lldesc[0];  

  const spi_dma_ctx_t *dma_ctx = spi_bus_get_dma_ctx(SPI2_HOST);
  int tx_chan_id = -1;
  esp_err_t chan_err = gdma_get_channel_id(dma_ctx->tx_dma_chan, &tx_chan_id);
  if (chan_err != ESP_OK || tx_chan_id < 0) {
    ESP_LOGE(TAG, "gdma_get_channel_id failed: %s (%d)", esp_err_to_name(chan_err), (int)chan_err);
    return (chan_err != ESP_OK) ? chan_err : ESP_ERR_INVALID_STATE;
  }

  gdma_ll_tx_start(&GDMA, tx_chan_id);

  return err;
}

// This stops the DMA engine from sending data to the SPI periphal, but does NOT generate a SPI interrupt
// as the SPI periphal is in limbo waiting for data I presume.
esp_err_t spi_dma_transfer_loop_pause(void) 
{
  esp_err_t err = ESP_OK;

  ESP_LOGD(TAG, "Calling spi_dma_transfer_loop_pause()");    


  // Point to end, must ensure we give the SPI perip all the data it expects, so only
  // set the chain to null at the end of the payload dma descriptors.
  // If we don't do this, and don't send all the payload in DMA memory to the SPI device
  // we'll get a CPU fault / watchdog reset.
  dma_data_lldesc[dma_lldesc_required-1].eof = 1;
  dma_data_lldesc[dma_lldesc_required-1].qe.stqe_next = NULL;

  return err;
}
