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

// 16-bit CIE 1931 lookup table
// Maps 8-bit input (0-255) to 16-bit output (0-65535)
static const uint16_t lumConvTab_16bit[256] = {
      0,    28,    57,    85,   114,   142,   171,   199,   228,   256,   285,   313,   342,   370,   399,   427,
    456,   484,   513,   541,   570,   598,   627,   658,   689,   721,   755,   789,   825,   861,   899,   937,
    977,  1018,  1060,  1103,  1147,  1192,  1239,  1287,  1336,  1386,  1437,  1490,  1544,  1599,  1656,  1714,
   1773,  1834,  1896,  1959,  2024,  2090,  2157,  2226,  2297,  2369,  2442,  2517,  2593,  2671,  2751,  2832,
   2914,  2999,  3085,  3172,  3261,  3352,  3444,  3538,  3634,  3732,  3831,  3932,  4035,  4139,  4245,  4354,
   4464,  4575,  4689,  4804,  4922,  5041,  5162,  5285,  5410,  5537,  5666,  5797,  5930,  6065,  6202,  6341,
   6482,  6626,  6771,  6918,  7068,  7220,  7373,  7529,  7687,  7848,  8010,  8175,  8342,  8512,  8683,  8857,
   9033,  9212,  9393,  9576,  9762,  9949, 10140, 10333, 10528, 10725, 10926, 11128, 11333, 11541, 11751, 11963,
  12179, 12396, 12617, 12840, 13065, 13293, 13524, 13757, 13993, 14232, 14474, 14718, 14965, 15215, 15467, 15722,
  15980, 16241, 16505, 16771, 17041, 17313, 17588, 17866, 18147, 18431, 18717, 19007, 19300, 19596, 19894, 20196,
  20501, 20809, 21119, 21433, 21750, 22071, 22394, 22720, 23050, 23383, 23719, 24058, 24400, 24746, 25095, 25447,
  25802, 26161, 26523, 26888, 27257, 27629, 28004, 28383, 28765, 29151, 29540, 29932, 30328, 30728, 31131, 31537,
  31947, 32360, 32777, 33198, 33622, 34050, 34481, 34916, 35355, 35797, 36243, 36693, 37146, 37603, 38064, 38529,
  38997, 39469, 39945, 40425, 40908, 41396, 41887, 42382, 42881, 43384, 43891, 44401, 44916, 45435, 45957, 46484,
  47015, 47549, 48088, 48631, 49178, 49728, 50283, 50843, 51406, 51973, 52545, 53120, 53700, 54284, 54873, 55465,
  56062, 56663, 57269, 57878, 58492, 59111, 59733, 60360, 60992, 61627, 62268, 62912, 63561, 64215, 64873, 65535,
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
  const uint16_t config_reg = (uint16_t)(
      (ghost_elimination_ON << 14) | ((PANEL_SCAN_LINES - 1) << 8) |
      (gray_scale_14 << 7) | (gclk_multiplier_OFF << 6) | current_1);
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

*/

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
