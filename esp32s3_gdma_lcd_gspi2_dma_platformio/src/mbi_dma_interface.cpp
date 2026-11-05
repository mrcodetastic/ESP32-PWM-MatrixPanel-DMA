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

#ifdef OPTIMIZE_FOR_SPEED
// Transpose a 16x16 bit matrix in place. On entry, each value is one GPIO
// lane's 16-bit intensity; on exit, each value is one parallel DMA sample.
static void transpose16x16(uint16_t A[16]) {
    uint32_t j, k;
    uint16_t t;

    // Stage 1: Swap 8x8 bit blocks (distance 8)
    for (k = 0; k < 8; k++) {
        t = (A[k] ^ (A[k + 8] >> 8)) & 0x00FF;
        A[k] ^= t;
        A[k + 8] ^= (t << 8);
    }

    // Stage 2: Swap 4x4 bit blocks (distance 4)
    for (k = 0; k < 16; k += 8) {
        for (j = 0; j < 4; j++) {
            t = (A[k + j] ^ (A[k + j + 4] >> 4)) & 0x0F0F;
            A[k + j] ^= t;
            A[k + j + 4] ^= (t << 4);
        }
    }

    // Stage 3: Swap 2x2 bit blocks (distance 2)
    for (k = 0; k < 16; k += 4) {
        for (j = 0; j < 2; j++) {
            t = (A[k + j] ^ (A[k + j + 2] >> 2)) & 0x3333;
            A[k + j] ^= t;
            A[k + j + 2] ^= (t << 2);
        }
    }

    // Stage 4: Swap individual bits (distance 1)
    for (k = 0; k < 16; k += 2) {
        t = (A[k] ^ (A[k + 1] >> 1)) & 0x5555;
        A[k] ^= t;
        A[k + 1] ^= (t << 1);
    }
}
#endif


  // CIE - Lookup table for converting between perceived LED brightness and PWM
  // https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e
static const uint16_t lumConvTab_8bit[256] = {
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

  resetFrameBuffer();

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
/*

[  7368][W][Matrix.h:166] update(): [Matrix.h] Effective frame rate: 13.70 frames/sec
[  7376][I][main.cpp:100] loop(): [main] profile render=14757 us, render+update=113816 us
[  7458][I][mbi_dma_interface.cpp:229] sendCommand(): [mbi_dma_interface] sendVsync: submitting 216 words; LAT high in 2 word(s), first=100 last=101
[  7471][W][Matrix.h:154] update(): [Matrix.h] update() timing: framebuffer flush=48673 us, greyscale DMA frame=6474 us, GPSPI2 stop=4461 us, VSYNC=13304 us, GPSPI2 start=52 us, recovery=0 us, total=72968 us

*/
/*
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
*/

// Convert one set of 12 parallel intensity lanes into a contiguous 16-sample
// block using a bit-matrix transpose, rather than assembling each sample bit
// by bit.

// ESP32-S3 has no bit-manipulation instructions so we have to do this in software.

/*
[ 48783][W][Matrix.h:166] update(): [Matrix.h] Effective frame rate: 17.55 frames/sec
[ 48790][I][main.cpp:100] loop(): [main] profile render=14759 us, render+update=97857 us
[ 48857][I][mbi_dma_interface.cpp:229] sendCommand(): [mbi_dma_interface] sendVsync: submitting 216 words; LAT high in 2 word(s), first=100 last=101
[ 48870][W][Matrix.h:154] update(): [Matrix.h] update() timing: framebuffer flush=32837 us, greyscale DMA frame=6481 us, GPSPI2 stop=4271 us, VSYNC=13337 us, GPSPI2 start=52 us, recovery=0 us, total=56981 us

*/
void MbiDmaInterface::renderLinearBufferToDMA_8bit(
    const Rgb8 (*pixel_buffer)[PANEL_PHY_RES_X + 2]) {
  if (!_ready || pixel_buffer == nullptr) {
    return;
  }

  constexpr int row_groups =
      (PANEL_PHY_RES_Y + PANEL_SCAN_LINES - 1) / PANEL_SCAN_LINES;

  for (int row = 0; row < PANEL_SCAN_LINES; ++row) {
    const size_t row_offset =
        static_cast<size_t>(row) * MBI_ENTRIES_PER_ROW;

    for (int x = 0; x < PANEL_MBI_RES_X; ++x) {
      uint16_t lanes[MBI_BITS_PER_CHANNEL] = {};

      for (int group = 0; group < row_groups; ++group) {
        const int y = group * PANEL_SCAN_LINES + row;
        if (y >= PANEL_PHY_RES_Y) {
          continue;
        }

        const Rgb8 &pixel = pixel_buffer[y][x];
        const int lane = group * MBI_COLOUR_BITS_PER_GROUP;
        lanes[15 - lane] = lumConvTab_8bit[pixel.g] << 8;
        lanes[14 - lane] = lumConvTab_8bit[pixel.b] << 8;
        lanes[13 - lane] = lumConvTab_8bit[pixel.r] << 8;
      }

      transpose16x16(lanes);

      const size_t output_offset =
          row_offset +
          static_cast<size_t>(x % PANEL_MBI_LED_CHANS) * MBI_ENTRIES_PER_IC +
          static_cast<size_t>(x / PANEL_MBI_LED_CHANS) * MBI_BITS_PER_CHANNEL;
      const bool latch_after_block =
          x / PANEL_MBI_LED_CHANS == PANEL_MBI_CHAIN_LEN - 1;

      // Hack, only populate top 8 bits of 16 bit transposed output, all other bits should be zero
      for (int sample = 0; sample < MBI_BITS_PER_CHANNEL/2; ++sample) {

        // Latchs pre-written when framebuffer is setup, and we make sure we DON'T wipe the framebuffer!
        /*
        const uint16_t latch =
            latch_after_block && sample == MBI_BITS_PER_CHANNEL - 1
                ? static_cast<uint16_t>(BIT_LAT)
                : 0;
        */
        _frame_buffer[output_offset + sample] =
            static_cast<ESP32_GREY_DMA_STORAGE_TYPE>(lanes[sample]); // | latch);
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

// LAT is fixed for every greyscale frame. Keep it in the DMA buffer so each
// send does not need to walk and rewrite the complete frame before transmission.
void MbiDmaInterface::restoreFrameLatchBits() {
  for (size_t row = 0; row < PANEL_SCAN_LINES; ++row) {
    for (size_t chan = 0; chan < PANEL_MBI_LED_CHANS; ++chan) {
      const size_t latch_pos = row * MBI_ENTRIES_PER_ROW +
                               chan * MBI_ENTRIES_PER_IC +
                               MBI_ENTRIES_PER_IC - 1;
      _frame_buffer[latch_pos] |= (ESP32_GREY_DMA_STORAGE_TYPE)BIT_LAT;
    }
  }
}


void MbiDmaInterface::resetFrameBuffer() {
  if (_frame_buffer == nullptr) {
    return;
  }
  memset(_frame_buffer, 0, _frame_bytes);
  restoreFrameLatchBits();

}
