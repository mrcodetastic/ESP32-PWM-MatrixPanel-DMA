/******************************************************************************************
 * @file        mbi_dma_interface.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       LCD_CAM / GDMA parallel bus + MBI5153 command sequencing.
 *
 * Implementation of the MbiDmaInterface class (see include/mbi_dma_interface.h).
 * All the code here used to live inline in Matrix.h; it has been moved out
 * unchanged apart from being wrapped in a class so the software framebuffer /
 * drawing layer no longer has to know about DMA buffers, descriptors or MBI
 * command framing.
 ******************************************************************************************/

#include "mbi_dma_interface.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <string.h>

static const char *const TAG = "mbi_dma_interface";
static constexpr size_t MBI_CMD_TRAILING_LOW_CLOCKS = 16;

// LAT is fixed for every greyscale frame. Keep it in the DMA buffer so each
// send does not need to walk and rewrite the complete frame before transmission.
static void restoreFrameLatchBits(ESP32_GREY_DMA_STORAGE_TYPE *frame_buffer) {
  for (size_t row = 0; row < PANEL_SCAN_LINES; ++row) {
    for (size_t chan = 0; chan < PANEL_MBI_LED_CHANS; ++chan) {
      const size_t latch_pos = row * MBI_ENTRIES_PER_ROW +
                               chan * MBI_ENTRIES_PER_IC +
                               MBI_ENTRIES_PER_IC - 1;
      frame_buffer[latch_pos] |= (ESP32_GREY_DMA_STORAGE_TYPE)BIT_LAT;
    }
  }
}

// CIE - Generate lookup table(s) for converting between perceived LED brightness and PWM
// https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA/tree/master/tools

// 14-bit CIE 1931 lookup table
// Maps 8-bit input (0-255) to 14-bit output (0-16383)
static const uint16_t lumConvTab_14bit[256] = {
      0,     7,    14,    21,    28,    36,    43,    50,    57,    64,    71,    78,    85,    93,   100,   107,
    114,   121,   128,   135,   142,   149,   157,   164,   172,   180,   189,   197,   206,   215,   225,   234,
    244,   254,   265,   276,   287,   298,   310,   322,   334,   346,   359,   373,   386,   400,   414,   428,
    443,   458,   474,   490,   506,   522,   539,   557,   574,   592,   610,   629,   648,   668,   688,   708,
    729,   750,   771,   793,   815,   838,   861,   885,   909,   933,   958,   983,  1009,  1035,  1061,  1088,
   1116,  1144,  1172,  1201,  1230,  1260,  1290,  1321,  1353,  1384,  1417,  1449,  1482,  1516,  1550,  1585,
   1621,  1656,  1693,  1729,  1767,  1805,  1843,  1882,  1922,  1962,  2003,  2044,  2085,  2128,  2171,  2214,
   2258,  2303,  2348,  2394,  2440,  2487,  2535,  2583,  2632,  2681,  2731,  2782,  2833,  2885,  2938,  2991,
   3045,  3099,  3154,  3210,  3266,  3323,  3381,  3439,  3498,  3558,  3618,  3679,  3741,  3803,  3867,  3930,
   3995,  4060,  4126,  4193,  4260,  4328,  4397,  4466,  4536,  4607,  4679,  4752,  4825,  4899,  4973,  5049,
   5125,  5202,  5280,  5358,  5437,  5517,  5598,  5680,  5762,  5845,  5929,  6014,  6100,  6186,  6273,  6361,
   6450,  6540,  6630,  6722,  6814,  6907,  7001,  7095,  7191,  7287,  7385,  7483,  7582,  7682,  7782,  7884,
   7986,  8090,  8194,  8299,  8405,  8512,  8620,  8729,  8838,  8949,  9060,  9173,  9286,  9400,  9516,  9632,
   9749,  9867,  9986, 10106, 10227, 10348, 10471, 10595, 10720, 10845, 10972, 11100, 11228, 11358, 11489, 11620,
  11753, 11887, 12021, 12157, 12294, 12432, 12570, 12710, 12851, 12993, 13136, 13279, 13424, 13570, 13718, 13866,
  14015, 14165, 14317, 14469, 14622, 14777, 14933, 15089, 15247, 15406, 15566, 15727, 15890, 16053, 16217, 16383,
};

// The lut table above, but scaled to 14-bit MBI5153 greyscale words (ie. shifted left by 2 bits), so we don't have to compute this EVERY TIME we sent a pixel to the MBI5153 greyscale DMA buffer. This is a performance optimization.
const uint16_t lumConvTab_14bit_mbi[256] = {
      0<<2,    7<<2,   14<<2,   21<<2,   28<<2,   36<<2,   43<<2,   50<<2,   57<<2,   64<<2,   71<<2,   78<<2,   85<<2,   93<<2,  100<<2,  107<<2,
    114<<2,  121<<2,  128<<2,  135<<2,  142<<2,  149<<2,  157<<2,  164<<2,  172<<2,  180<<2,  189<<2,  197<<2,  206<<2,  215<<2,  225<<2,  234<<2,
    244<<2,  254<<2,  265<<2,  276<<2,  287<<2,  298<<2,  310<<2,  322<<2,  334<<2,  346<<2,  359<<2,  373<<2,  386<<2,  400<<2,  414<<2,  428<<2,
    443<<2,  458<<2,  474<<2,  490<<2,  506<<2,  522<<2,  539<<2,  557<<2,  574<<2,  592<<2,  610<<2,  629<<2,  648<<2,  668<<2,  688<<2,  708<<2,
    729<<2,  750<<2,  771<<2,  793<<2,  815<<2,  838<<2,  861<<2,  885<<2,  909<<2,  933<<2,  958<<2,  983<<2, 1009<<2, 1035<<2, 1061<<2, 1088<<2,
   1116<<2, 1144<<2, 1172<<2, 1201<<2, 1230<<2, 1260<<2, 1290<<2, 1321<<2, 1353<<2, 1384<<2, 1417<<2, 1449<<2, 1482<<2, 1516<<2, 1550<<2, 1585<<2,
   1621<<2, 1656<<2, 1693<<2, 1729<<2, 1767<<2, 1805<<2, 1843<<2, 1882<<2, 1922<<2, 1962<<2, 2003<<2, 2044<<2, 2085<<2, 2128<<2, 2171<<2, 2214<<2,
   2258<<2, 2303<<2, 2348<<2, 2394<<2, 2440<<2, 2487<<2, 2535<<2, 2583<<2, 2632<<2, 2681<<2, 2731<<2, 2782<<2, 2833<<2, 2885<<2, 2938<<2, 2991<<2,
   3045<<2, 3099<<2, 3154<<2, 3210<<2, 3266<<2, 3323<<2, 3381<<2, 3439<<2, 3498<<2, 3558<<2, 3618<<2, 3679<<2, 3741<<2, 3803<<2, 3867<<2, 3930<<2,
   3995<<2, 4060<<2, 4126<<2, 4193<<2, 4260<<2, 4328<<2, 4397<<2, 4466<<2, 4536<<2, 4607<<2, 4679<<2, 4752<<2, 4825<<2, 4899<<2, 4973<<2, 5049<<2,
   5125<<2, 5202<<2, 5280<<2, 5358<<2, 5437<<2, 5517<<2, 5598<<2, 5680<<2, 5762<<2, 5845<<2, 5929<<2, 6014<<2, 6100<<2, 6186<<2, 6273<<2, 6361<<2,
   6450<<2, 6540<<2, 6630<<2, 6722<<2, 6814<<2, 6907<<2, 7001<<2, 7095<<2, 7191<<2, 7287<<2, 7385<<2, 7483<<2, 7582<<2, 7682<<2, 7782<<2, 7884<<2,
   7986<<2, 8090<<2, 8194<<2, 8299<<2, 8405<<2, 8512<<2, 8620<<2, 8729<<2, 8838<<2, 8949<<2, 9060<<2, 9173<<2, 9286<<2, 9400<<2, 9516<<2, 9632<<2,
   9749<<2, 9867<<2, 9986<<2, 10106<<2, 10227<<2, 10348<<2, 10471<<2, 10595<<2, 10720<<2, 10845<<2, 10972<<2, 11100<<2, 11228<<2, 11358<<2, 11489<<2, 11620<<2,
  11753<<2, 11887<<2, 12021<<2, 12157<<2, 12294<<2, 12432<<2, 12570<<2, 12710<<2, 12851<<2, 12993<<2, 13136<<2, 13279<<2, 13424<<2, 13570<<2, 13718<<2, 13866<<2,
  14015<<2, 14165<<2, 14317<<2, 14469<<2, 14622<<2, 14777<<2, 14933<<2, 15089<<2, 15247<<2, 15406<<2, 15566<<2, 15727<<2, 15890<<2, 16053<<2, 16217<<2, 16383<<2,
};



// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

MbiDmaInterface::~MbiDmaInterface() {
  // Bus_Parallel16's own destructor releases the LCD_CAM/GDMA resources.
  if (_frame_buffer) {
    heap_caps_free(_frame_buffer);
    _frame_buffer = nullptr;
  }
  if (_cmd_buffer) {
    heap_caps_free(_cmd_buffer);
    _cmd_buffer = nullptr;
  }
}

esp_err_t MbiDmaInterface::begin() {
  // Step 1) Allocate raw buffer space for MBI5153 greyscale / MBI chip command / pixel memory
  _frame_words = (PANEL_SCAN_LINES * PANEL_MBI_RES_X * MBI_BITS_PER_CHANNEL);
  _frame_bytes = sizeof(ESP32_GREY_DMA_STORAGE_TYPE) * _frame_words;  // add some extra blank data at the end for time to latch if we're updating the greyscale buffers
  _frame_bytes = _frame_bytes + 128;
  ESP_LOGD(TAG, "Allocating greyscale DMA memory buffer. Size of memory required: %lu bytes.", _frame_bytes);

  // Malloc Greyscale DMA Memory
  _frame_buffer = (ESP32_GREY_DMA_STORAGE_TYPE *)heap_caps_calloc(1, _frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (_frame_buffer == nullptr) {
    ESP_LOGE(TAG, "Could not allocate greyscale DMA buffer");
    return ESP_ERR_NO_MEM;
  }
  restoreFrameLatchBits(_frame_buffer);

  // Dedicated scratch buffer for command / Vsync payloads so they never
  // clobber the live greyscale framebuffer.
  _cmd_buffer = (ESP32_GREY_DMA_STORAGE_TYPE *)heap_caps_calloc(
      1, MBI_CMD_BUFFER_ENTRIES * sizeof(ESP32_GREY_DMA_STORAGE_TYPE),
      (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
  if (_cmd_buffer == nullptr) {
    ESP_LOGE(TAG, "Could not allocate command DMA buffer");
    return ESP_ERR_NO_MEM;
  }

  // Setup LCD DMA and Output to GPIO
  auto bus_cfg = _bus.config();
  bus_cfg.pin_wr = MBI_DCLK;  // DCLK Pin
  bus_cfg.invert_pclk = false;
  bus_cfg.pin_d0 = MBI_G1;
  bus_cfg.pin_d1 = MBI_B1;
  bus_cfg.pin_d2 = MBI_R1;
  bus_cfg.pin_d3 = MBI_G2;
  bus_cfg.pin_d4 = MBI_B2;
  bus_cfg.pin_d5 = MBI_R2;
  bus_cfg.pin_d6 = MBI_G3;
  bus_cfg.pin_d7 = MBI_B3;
  bus_cfg.pin_d8 = MBI_R3;
  bus_cfg.pin_d9 = MBI_G4;
  bus_cfg.pin_d10 = MBI_B4;
  bus_cfg.pin_d11 = MBI_R4;
  bus_cfg.pin_d12 = MBI_LAT;  // Latch
  bus_cfg.pin_d13 = -1;       // DCLK potentially if we need to manually generate
  bus_cfg.pin_d14 = -1;
  bus_cfg.pin_d15 = -1;

  _bus.config(bus_cfg);
  ESP_LOGI(TAG, "LAT routing: LCD data bit 12 -> GPIO%d", (int)MBI_LAT);
  const esp_err_t lcd_setup_result = _bus.setup_lcd_dma_periph();
  if (lcd_setup_result != ESP_OK) {
    ESP_LOGE(TAG, "LCD_CAM/GDMA setup failed: %s (%d)",
             esp_err_to_name(lcd_setup_result), (int)lcd_setup_result);
    return lcd_setup_result;
  }

  _ready = true;
  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Command plumbing
// ---------------------------------------------------------------------------

esp_err_t MbiDmaInterface::sendCommand(const char *what, size_t words) {
  const char *label = what ? what : "sendCommand";
  if (!_ready || _cmd_buffer == nullptr) {
    ESP_LOGE(TAG, "%s: DMA bus/buffer not initialised", label);
    return ESP_ERR_INVALID_STATE;
  }
  if (words == 0 || words > MBI_CMD_BUFFER_ENTRIES - MBI_CMD_TRAILING_LOW_CLOCKS) {
    ESP_LOGE(TAG, "%s: invalid command payload length (%u words)",
             label, (unsigned)words);
    return ESP_ERR_INVALID_SIZE;
  }

  memset(&_cmd_buffer[words], 0,
         MBI_CMD_TRAILING_LOW_CLOCKS * sizeof(*_cmd_buffer));
  const size_t transfer_words = words + MBI_CMD_TRAILING_LOW_CLOCKS;

  size_t lat_count = 0;
  size_t first_lat_pos = words;
  size_t last_lat_pos = words;
  for (size_t pos = 0; pos < words; ++pos) {
    if (_cmd_buffer[pos] & BIT_LAT) {
      if (lat_count == 0) first_lat_pos = pos;
      last_lat_pos = pos;
      ++lat_count;
    }
  }
  ESP_LOGI(TAG, "%s: submitting %u words; LAT high in %u word(s), first=%u last=%u",
           label, (unsigned)transfer_words, (unsigned)lat_count,
           (unsigned)first_lat_pos, (unsigned)last_lat_pos);

  const esp_err_t ret = _bus.send_stuff_once(
      _cmd_buffer, transfer_words * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: transfer failed: %s (%d)",
             label, esp_err_to_name(ret), (int)ret);
  }
  return ret;
}

void MbiDmaInterface::clearCommandBuffer() {
  if (_cmd_buffer != nullptr) {
    memset(_cmd_buffer, 0,
           MBI_CMD_BUFFER_ENTRIES * sizeof(*_cmd_buffer));
  }
}

/* Pre-activation command - sent before sending configuration register data */
esp_err_t MbiDmaInterface::preActive() {
  if (!_ready) return ESP_ERR_INVALID_STATE;
  clearCommandBuffer();
  for (int clock = 0; clock < MBI_PRE_ACTIVE_LAT_CLOCKS; ++clock) {
    _cmd_buffer[clock] = (uint16_t)BIT_LAT;
  }
  return sendCommand("preActive", MBI_PRE_ACTIVE_LAT_CLOCKS);
}

/* Vertical Sync - updates frame data on outputs, used in conjunction with vertical scan

   9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
   7ns = 142 Mhz clock speed...

   After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display
   buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives
   the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.
*/
esp_err_t MbiDmaInterface::sendVsync() {
  if (!_ready) return ESP_ERR_INVALID_STATE;
  clearCommandBuffer();
  const size_t latch_start = MBI_VSYNC_PAYLOAD_LEN / 2;
  for (int clock = 0; clock < MBI_VSYNC_LAT_CLOCKS; ++clock) {
    _cmd_buffer[latch_start + clock] = (uint16_t)BIT_LAT;
  }
  return sendCommand("sendVsync", MBI_VSYNC_PAYLOAD_LEN);
}

esp_err_t MbiDmaInterface::softReset() {
  /* "Software reset" command makes MBI5153 go back to the initial state except configuration register value. After this
  command is received, the output channels will be turned off and will display again with last gray-scale value after
  new "Vsync" command is received.
  */
  if (!_ready) return ESP_ERR_INVALID_STATE;
  clearCommandBuffer();
  for (int clock = 0; clock < MBI_SOFT_RESET_LAT_CLOCKS; ++clock) {
    _cmd_buffer[clock] = (uint16_t)BIT_LAT;
  }
  return sendCommand("softReset", MBI_SOFT_RESET_LAT_CLOCKS);
}

esp_err_t MbiDmaInterface::sendConfigReg1() {
  if (!_ready) return ESP_ERR_INVALID_STATE;
  clearCommandBuffer();
/*  
  const uint16_t config_reg = (uint16_t)(
      (ghost_elimination_ON << 14) | ((PANEL_SCAN_LINES - 1) << 8) |
      (gray_scale_14 << 7) | (gclk_multiplier_OFF << 6) | current_1);

*/      
const uint16_t config_reg = (uint16_t)(
      (ghost_elimination_ON << 14) | ((PANEL_SCAN_LINES - 1) << 8) |
      (gray_scale_14 << 7) | (gclk_multiplier_ON << 6) | current_1);


  const size_t payload_length = PANEL_MBI_CHAIN_LEN * MBI_BITS_PER_CHANNEL;
  for (size_t index = 0; index < payload_length; ++index) {
    const size_t chip = index / MBI_BITS_PER_CHANNEL;
    const int bit = MBI_BITS_PER_CHANNEL - 1 -
                    (index % MBI_BITS_PER_CHANNEL);
    uint16_t word = (config_reg & (1U << bit)) ? (uint16_t)BIT_ALL_RGB : 0;
    if (chip == PANEL_MBI_CHAIN_LEN - 1 && bit < MBI_CONFIG1_LATCH_TRIGGER) {
      word |= (uint16_t)BIT_LAT;
    }
    _cmd_buffer[index] = word;
  }
  return sendCommand("sendConfigReg1", payload_length);
}

esp_err_t MbiDmaInterface::sendConfigReg2() {
  if (!_ready) return ESP_ERR_INVALID_STATE;
  clearCommandBuffer();
  constexpr uint16_t config_reg = 0b1001000000011110;
  const size_t payload_length = PANEL_MBI_CHAIN_LEN * MBI_BITS_PER_CHANNEL;
  for (size_t index = 0; index < payload_length; ++index) {
    const size_t chip = index / MBI_BITS_PER_CHANNEL;
    const int bit = MBI_BITS_PER_CHANNEL - 1 -
                    (index % MBI_BITS_PER_CHANNEL);
    uint16_t word = (config_reg & (1U << bit)) ? (uint16_t)BIT_ALL_RGB : 0;
    if (chip == PANEL_MBI_CHAIN_LEN - 1 && bit < MBI_CONFIG2_LATCH_TRIGGER) {
      word |= (uint16_t)BIT_LAT;
    }
    _cmd_buffer[index] = word;
  }
  return sendCommand("sendConfigReg2", payload_length);
}

esp_err_t MbiDmaInterface::configurePanel() {
  if (!_ready) return ESP_ERR_INVALID_STATE;
  
  esp_err_t ret = softReset();
  if (ret != ESP_OK) return ret;

  ret = preActive();
  if (ret != ESP_OK) return ret;

  ret = sendConfigReg1();
  if (ret != ESP_OK) return ret;

  ret = softReset();
  if (ret != ESP_OK) return ret;

  
  ret = preActive();
  if (ret != ESP_OK) return ret;
  
  ret = sendConfigReg2();
  if (ret != ESP_OK) return ret;

  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Frame data
// ---------------------------------------------------------------------------

/*

MBI5152 Application Note V1.00- EN

Section 2: The Setting of Gray Scale  

The setting of gray scale data describes as below. 
1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M (M≦16) 
2. The data sequence of cascaded IC is ICnICn-1…. IC2IC1. 
3. The data sequence of each channel is ch15ch14…. ch0. 
4. The data length of each channel is 16-bits, and the default PWM mode is 16-bits. The sequence of gray 
sacle is bit15bit14bit13…bit0 as figure 4 shows. The 14-bits gray scale can be set through Bit[7]=1 
in configuration register 1, and the sequence of gray scale data is bit13bit12… bit0 0 0, the 
last 2-bits (LSB) are set to “0”. 
5. The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data. 
6. LE executes the data latch to send gray scale data into SRAM. Each 16xN bits data needs a “data latch 
command”, where N means the number of cascaded driver.  
7. After the last data latch, it needs at least 50 GCLKs to read the gray scale data into internal display buffer 
before the Vsync command comes.  
8. Display is updated immediately when MBI5152 receives the Vsync signal.  
9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signal. 
10. The period of dead time (ie. The 1025th GCLK) must be larger than 100ns.

The gray scale data needs the GCLK to save the data into SRAM. The frequency of GCLK must be higher
than 20% of DCLK to get the correct data.


// MBI5153
After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display
buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives
the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.

The DMA framing for the above lives in MbiDmaInterface (include/mbi_dma_interface.h).

*/

#ifdef OPTIMIZE_FOR_SPEED
void MbiDmaInterface::renderLinearBufferToDMA(
    const Rgb16 (*pixel_buffer)[PANEL_PHY_RES_X + 2]) {
  if (!_ready || pixel_buffer == nullptr) {
    return;
  }

  const int total_cols = PANEL_PHY_RES_X + 2;
  const int num_ics = (total_cols + PANEL_MBI_LED_CHANS - 1) / PANEL_MBI_LED_CHANS;

  // Loop through each physical row
  for (uint16_t y = 0; y < PANEL_PHY_RES_Y; ++y) {
    uint16_t row_group = y / PANEL_SCAN_LINES; // Compiler optimizes small constants/powers of 2 well
    uint16_t y_normalised = y % PANEL_SCAN_LINES;

    uint16_t _colourbitsoffset = row_group * MBI_COLOUR_BITS_PER_GROUP;
    uint16_t _colourbitsclear  = ~(0b111 << _colourbitsoffset);
    uint16_t g_gpio_bitmask = BIT_G1 << _colourbitsoffset;
    uint16_t b_gpio_bitmask = BIT_B1 << _colourbitsoffset;
    uint16_t r_gpio_bitmask = BIT_R1 << _colourbitsoffset;

    uint32_t base_row_offset = MBI_ENTRIES_PER_ROW * y_normalised;

    // Iterate by IC and Channel to completely eliminate division/modulo math
    for (int ic = 0; ic < num_ics; ++ic) {
      int ic_base_x = ic * PANEL_MBI_LED_CHANS;
      uint32_t ic_bit_base = base_row_offset + (ic * MBI_BITS_PER_CHANNEL);

      for (int chan = 0; chan < PANEL_MBI_LED_CHANS; ++chan) {
        int x = ic_base_x + chan;
        if (x >= total_cols) {
          break;
        }

        // Fetch pre-converted pixel directly from Matrix's framebuffer.
        const Rgb16 pix = pixel_buffer[y][x];
        uint16_t r_val = pix.r;
        uint16_t g_val = pix.g;
        uint16_t b_val = pix.b;

        // Calculate starting position using pure addition and multiplication (no division/mod)
        uint32_t bit_start_pos = ic_bit_base + (chan * MBI_ENTRIES_PER_IC);

        // 16-bit bitplane write loop
        for (uint16_t mask = 0x8000; mask != 0; mask >>= 1) {
          const uint16_t rgb_bits = (uint16_t)(
              ((0U - (uint32_t)((g_val & mask) != 0)) & g_gpio_bitmask) |
              ((0U - (uint32_t)((b_val & mask) != 0)) & b_gpio_bitmask) |
              ((0U - (uint32_t)((r_val & mask) != 0)) & r_gpio_bitmask));

          _frame_buffer[bit_start_pos] =
              (_frame_buffer[bit_start_pos] & _colourbitsclear) | rgb_bits;
          ++bit_start_pos;
        }
      }
    }
  }
}

// Asked GPT Luna to optimise renderLinearBufferToDMA
void MbiDmaInterface::renderLinearBufferToDMAv2(
    const Rgb16 (*pixel_buffer)[PANEL_PHY_RES_X + 2]) {
  if (!_ready || pixel_buffer == nullptr) {
    return;
  }

  constexpr int total_cols = PANEL_PHY_RES_X + 2;
  constexpr int num_ics =
      (total_cols + PANEL_MBI_LED_CHANS - 1) / PANEL_MBI_LED_CHANS;
  constexpr int row_groups =
      (PANEL_PHY_RES_Y + PANEL_SCAN_LINES - 1) / PANEL_SCAN_LINES;

  uint16_t g_masks[row_groups];
  uint16_t b_masks[row_groups];
  uint16_t r_masks[row_groups];

  for (int group = 0; group < row_groups; ++group) {
    const int offset = group * MBI_COLOUR_BITS_PER_GROUP;
    g_masks[group] = static_cast<uint16_t>(BIT_G1 << offset);
    b_masks[group] = static_cast<uint16_t>(BIT_B1 << offset);
    r_masks[group] = static_cast<uint16_t>(BIT_R1 << offset);
  }

  for (int row = 0; row < PANEL_SCAN_LINES; ++row) {
    const uint32_t row_offset =
        static_cast<uint32_t>(row) * MBI_ENTRIES_PER_ROW;

    for (int ic = 0; ic < num_ics; ++ic) {
      const int x_base = ic * PANEL_MBI_LED_CHANS;
      const uint32_t ic_offset =
          row_offset + static_cast<uint32_t>(ic) * MBI_BITS_PER_CHANNEL;

      for (int chan = 0; chan < PANEL_MBI_LED_CHANS; ++chan) {
        const int x = x_base + chan;
        if (x >= total_cols) {
          break;
        }

        uint16_t red[row_groups] = {};
        uint16_t green[row_groups] = {};
        uint16_t blue[row_groups] = {};

        for (int group = 0; group < row_groups; ++group) {
          const int y = group * PANEL_SCAN_LINES + row;
          if (y < PANEL_PHY_RES_Y) {
            const Rgb16 &pixel = pixel_buffer[y][x];
            red[group] = pixel.r;
            green[group] = pixel.g;
            blue[group] = pixel.b;
          }
        }

        uint32_t output_offset =
            ic_offset + static_cast<uint32_t>(chan) * MBI_ENTRIES_PER_IC;

        for (uint16_t mask = 0x8000; mask != 0; mask >>= 1) {
          uint16_t output = 0;

          for (int group = 0; group < row_groups; ++group) {
            if (green[group] & mask) output |= g_masks[group];
            if (blue[group] & mask) output |= b_masks[group];
            if (red[group] & mask) output |= r_masks[group];
          }

          // LAT is at the final bit of each channel's IC chain.
          if (ic == PANEL_MBI_CHAIN_LEN - 1 && mask == 1) {
            output |= BIT_LAT;
          }

          _frame_buffer[output_offset++] =
              static_cast<ESP32_GREY_DMA_STORAGE_TYPE>(output);
        }
      }
    }
  }
}




#else

// Specific for the 14 bit LUT lookup table, which is the native hardware range for the MBI5153.
void MbiDmaInterface::setPixel(uint8_t x, uint8_t y, uint8_t _r_data, uint8_t _g_data, uint8_t _b_data) {
  if (!_ready) {
    return;
  }

  if (x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
    return;
  }

// Directly grab the native 14-bit perceptual value and shift left by 2 for full scale
  uint16_t r_val = (uint16_t)(lumConvTab_14bit[_r_data] << 2);
  uint16_t g_val = (uint16_t)(lumConvTab_14bit[_g_data] << 2);
  uint16_t b_val = (uint16_t)(lumConvTab_14bit[_b_data] << 2);

  // --- QUICK TEST HACK (Remove when done testing) ---
  // r_val = 0x0000;
  // g_val = 0x0000;
  // b_val = 0xFFFF;
  // ---------------------------------------------------

  x += 2;  // offset for missing pixels on the left

  // Calculate bitmasks
  uint16_t _colourbitsoffset = (y / PANEL_SCAN_LINES) * MBI_COLOUR_BITS_PER_GROUP;
  uint16_t _colourbitsclear  = ~(0b111 << _colourbitsoffset);  // invert

  uint16_t g_gpio_bitmask = BIT_G1 << _colourbitsoffset;  // bit 0
  uint16_t b_gpio_bitmask = BIT_B1 << _colourbitsoffset;  // bit 1
  uint16_t r_gpio_bitmask = BIT_R1 << _colourbitsoffset;  // bit 2

  // Map pixel coordinates to channels and ICs
  int y_normalised = y % PANEL_SCAN_LINES;

  // Calculate buffer start position using the inverted tx_chan and tx_ic coordinates
  int bit_start_pos = (MBI_ENTRIES_PER_ROW * y_normalised)
                    + ((x % PANEL_MBI_LED_CHANS) * MBI_ENTRIES_PER_IC)
                    + ((x / PANEL_MBI_LED_CHANS) * MBI_BITS_PER_CHANNEL);

  // Update only this pixel's three RGB lanes. The LUT values are stored as
  // bits 15..0 in transmission order; LAT and the other panel quarters are
  // preserved in the existing DMA words.
  for (uint16_t mask = 0x8000; mask != 0; mask >>= 1) {
    const uint16_t rgb_bits = (uint16_t)(
        ((0U - (uint32_t)((g_val & mask) != 0)) & g_gpio_bitmask) |
        ((0U - (uint32_t)((b_val & mask) != 0)) & b_gpio_bitmask) |
        ((0U - (uint32_t)((r_val & mask) != 0)) & r_gpio_bitmask));
    _frame_buffer[bit_start_pos] =
        (_frame_buffer[bit_start_pos] & _colourbitsclear) | rgb_bits;
    ++bit_start_pos;
  }
}

#endif

esp_err_t MbiDmaInterface::sendFrame() {
  if (!_ready) {
    ESP_LOGE(TAG, "sendFrame(): DMA bus not initialised");
    return ESP_ERR_INVALID_STATE;
  }

  // The invariant LAT pattern is installed once in begin()/clearFrame(); pixel
  // updates only touch RGB lanes, so this buffer is ready to DMA as-is.
  const esp_err_t grey_ret = _bus.send_stuff_once(_frame_buffer, _frame_bytes, true);
  if (grey_ret != ESP_OK) {
    ESP_LOGE(TAG, "greyscale LCD DMA transfer failed: %s (%d)",
             esp_err_to_name(grey_ret), (int)grey_ret);
  }
  
  return grey_ret;
}


void MbiDmaInterface::clearFrame() {
  if (_frame_buffer == nullptr) {
    return;
  }
  memset(_frame_buffer, 0, _frame_bytes);
  restoreFrameLatchBits(_frame_buffer);
}
