#ifndef MATRIX_H
#define MATRIX_H

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_task_wdt.h>

#include <iostream>

#include "Arduino.h"
#include "app_constants.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_dma_parallel16.hpp"
//#include "MatrixSettings.h"
#include "sdkconfig.h"
#include "spi_dma_seg_tx_loop.h"
#include <array>
#include <GFX_Lite.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) \
  {                         \
    int16_t t = a;          \
    a = b;                  \
    b = t;                  \
  }
#endif

/*
  // MBI5152 Application Note V1.00- EN

  Section 2: The Setting of Gray Scale
  The setting of gray scale data describes as below.
  1. The sequence of input data starts from scan line 1 scan line 2….  scan line M-1 scan line M
  (M≦16)
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

*/

static const char *TAG = "Matrix.h";

// CIE - Lookup table for converting between perceived LED brightness and PWM
// https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e

// 8 bit lookup table as we're only passing through 8 bit colour data so pointless to scale it up
// to 14-bit or 16-bit values for mbi_set_pixel as it'll just take longer to load the data to the
// panel because of more bit iterations.
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

class Matrix : public GFX {

 public:
  Matrix() : GFX (PANEL_PHY_RES_X, PANEL_PHY_RES_Y) {  }

  ~Matrix() {
  }

  /***************************************** MBI STUFF  *****************************************/

  void initMatrix() 
  {
    // Step 1) Allocate raw buffer space for MBI5153 greyscale / MBI chip command /  pixel memory
    dma_grey_buffer_parallel_bit_length = ((PANEL_SCAN_LINES * PANEL_MBI_RES_X * 16));
    dma_grey_buffer_size = sizeof(ESP32_GREY_DMA_STORAGE_TYPE) * dma_grey_buffer_parallel_bit_length;  // add some extract blank data at the end for time to latch if we're updating the greyscale buffers
    ESP_LOGD(TAG, "Allocating greyscale DMA memory buffer. Size of memory required: %lu bytes.", dma_grey_buffer_size);

    // Malloc Greyscale / Command DMA Memory
    dma_grey_gpio_data = (ESP32_GREY_DMA_STORAGE_TYPE *)heap_caps_malloc(dma_grey_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    assert(dma_grey_gpio_data != nullptr);

    // Fill with zeros to start with
    memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);

    // Setup LCD DMA and Output to GPIO
    auto bus_cfg = dma_bus.config();
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

    dma_bus.config(bus_cfg);
    dma_bus.setup_lcd_dma_periph();

    // Setup SPI DMA Output for GCLK and Address Lines
    spi_setup();

    updateRegisters();

    update();

    initialized = true;    
 
  }

  void refreshMatrixConfig() {

    // spi_transfer_loop_stop();
    mbi_pre_active_dma();  // 14 clocks
    mbi_send_config_reg1_dma();

    // MBI Step 2) Some other register 2 hack to reduce ghosting.
    mbi_pre_active_dma();
    mbi_send_config_reg2_dma();
  }


/*
  // MBI5152 Application Note V1.00- EN
  
  <snipped>
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

*/
  void update() {

    assert(initialized);

    mbi_update_frame(true);
    spi_transfer_loop_stop();
    mbi_v_sync_dma();
    spi_transfer_loop_start();        

    //log_e("tsfr count: %d", dma_bus.get_transfer_count());

    memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);

  }

  void updateRegisters() 
  {
    spi_transfer_loop_start();  // start GCLK + Adress toggling

    // MBI Step 1) Set key registers, such as number of rows
    mbi_soft_reset_dma();  // 10 clocks
    mbi_pre_active_dma();  // 14 clocks
    mbi_send_config_reg1_dma();

    // MBI Step 2) Some other register 2 hack to reduce ghosting.
    mbi_pre_active_dma();
    mbi_send_config_reg2_dma();

    // MBI Step 3) Clean out any crap in the greyscale buffer
    mbi_soft_reset_dma();  // 10 clocks

    memset(dma_grey_gpio_data, 0, dma_grey_buffer_size);  // accidenty display configuration registers id we don't clear th
  }


  /***************************************** GFX STUFF  *****************************************/

  void setBrightness(uint8_t newBrightness) {}

  uint8_t getBrightness() const {
    return 255;
  }

  uint8_t getXResolution() {
    return PANEL_PHY_RES_X;
  }

  uint8_t getYResolution() {
    return PANEL_PHY_RES_Y;
  }


  // Always implement basic AdaFruit_GFX virtual functions
  void drawPixel(int16_t x, int16_t y, CRGB color) {
    mbi_set_pixel(x, y, color.red, color.green, color.blue);
  }

  void drawPixel(uint8_t x, uint8_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    mbi_set_pixel(x, y, r_data, g_data, b_data);    
  }


  // includes 565 to 888 conversion
  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    uint8_t r   = (color >> 8) & 0xf8;
    uint8_t  g  = (color >> 3) & 0xfc;
    uint8_t b   = (color << 3);
    r |= r >> 5;
    g |= g >> 6;
    b |= b >> 5;    

    mbi_set_pixel(x, y, r, g, b);
  }

  void writePixel(uint8_t x, uint8_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data) {
    mbi_set_pixel(x, y, r_data, g_data, b_data);
  }    

  // fillScreen will call the virtual implementation of fillRect

 protected:

  bool initialized = false;

  /***************************************** DMA STUFF  *****************************************/

  // D<A Data to send
  Bus_Parallel16 dma_bus;

  ESP32_GREY_DMA_STORAGE_TYPE *dma_grey_gpio_data;

  size_t dma_grey_buffer_parallel_bit_length; // Length in bits of the buffer -> sequance of 13 x 16 bits (2 bytes) sent in parallel = length value of 13
  size_t dma_grey_buffer_size; // length of buffer in memory used -> sequance of 13 x 16 bits (2 bytes) sent in parallel = value of 26 bytes

  /***************************************** MBI STUFF  *****************************************/

  void mbi_update_frame(bool configure_latches) { // chatGPT verion
      int counter = 0;
      for (int row = 0; row < PANEL_SCAN_LINES; row++) {
          for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) {
              for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) {
                  int bit_offset = 16;

                  // Check if this is the last IC to determine latch bit.
                  int latch = (ic == 4) ? 1 : 0;

                  while (bit_offset > 0) {
                      bit_offset--;

                      // Set BIT_LAT only on the last bit of the last IC
                      if (latch == 1 && bit_offset == 0) {
                          dma_grey_gpio_data[counter] |= BIT_LAT;
                      }

                      counter++;
                  }
              }
          }
      }

      // Send the greyscale data buffer via DMA
      dma_bus.send_stuff_once(dma_grey_gpio_data, dma_grey_buffer_size, true);
  }


  void mbi_update_frame_old(bool configure_latches) {

      int counter = 0;
      for (int row = 0; row < PANEL_SCAN_LINES; row++) {
        for (int chan = 0; chan < PANEL_MBI_LED_CHANS; chan++) {
          for (int ic = 0; ic < PANEL_MBI_CHAIN_LEN; ic++) {  // number of chained ICs

            // data latch on the last bit, when sending the last byte set latch=1
            int latch = 0;
            if (ic == 4) {
              latch = 1;
            }  // latch on last channel / ic

            int bit_offset = 16;
            while (bit_offset > 0)  // shift out MSB first per the documentation.
            {
              bit_offset--;  // start from 15

              if (latch == 1 && bit_offset == 0) {
                dma_grey_gpio_data[counter] |= BIT_LAT;
              }
              counter++;
            }
          }
        }
      }
   
    //log_d(TAG, "Sending greyscale data buffer out via LCD DMA.");
    dma_bus.send_stuff_once(dma_grey_gpio_data, dma_grey_buffer_size, true);  // sending payload hence TRUE

  }  // mbi_update_frame


  /*
    Gray Scale Mode and Scan-type S-PWM
    MBI5153 provides a selectable 14-bit or 13-bit gray scale by setting the configuration register1 bit [7]. The default
    value is set to ’0’ for 14-bit color depth. In 14-bit gray scale mode, users should still send 16-bit data with 2-bit ‘0’ in
    LSB bits. For example, {14’h1234, 2’h0}.
    MBI5153 has a smart S-PWM technology for scan type. With S-PWM, the total PWM cycles can be broken into MSB
    (Most Significant Bits) and LSB (Least Significant Bits) of gray scale cycles. The MSB information can be broken
    down into many refresh cycles to achieve overall same high bit resolution.

    The 16384 GCLKs (14-bit) PWM cycle of MBI5052/53 is divided into 32 sections, each section has 512 GCLKs.
  */
  void mbi_set_pixel(uint8_t x, uint8_t y, uint8_t _r_data, uint8_t _g_data, uint8_t _b_data) {
    
    if (x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
      return;
    }

    // CIE looked up
    uint8_t r_data = CIE[_r_data];
    uint8_t g_data = CIE[_g_data];
    uint8_t b_data = CIE[_b_data];

    x += 2;  // offset for missing pixels on the left

    // Calculate bitmasks
    uint16_t _colourbitsoffset = (y / PANEL_SCAN_LINES) * 3; // three is an important bit
    uint16_t _colourbitsclear  = ~(0b111 << _colourbitsoffset);  // invert

    uint16_t g_gpio_bitmask = BIT_G1 << _colourbitsoffset;  // bit 0
    uint16_t b_gpio_bitmask = BIT_B1 << _colourbitsoffset;  // bit 1
    uint16_t r_gpio_bitmask = BIT_R1 << _colourbitsoffset;  // bit 2    

    // Row offset + channel offset + individual IC LED offset
    // Calculate data array start position
    int y_normalised = y % PANEL_SCAN_LINES;  // Only have 20 rows of data...
    int bit_start_pos = (1280 * y_normalised) + ((x % 16) * 80) + ((x / 16) * 16);  

  /*
      MBI5153 provides a selectable 14-bit or 13-bit gray scale by setting the configuration register1 bit [7]. The default 
      value is set to ’0’ for 14-bit color depth. In 14-bit gray scale mode, users should still send 16-bit data with 2-bit ‘0’ in 
      LSB bits. For example, {14’h1234, 2’h0}. 
  */    

    // RGB colour data provided is only 8bits, so we'll fill it from bit 16 down to bit 8
    // 14-bit resolution = 16,384
    //  8-bit resolutoin = 255     

    int subpixel_colour_bit = 8;
    uint8_t mask;
    while (subpixel_colour_bit > 0)  // shift out MSB first per the documentation.
    {
      subpixel_colour_bit--;  // start from 7
      dma_grey_gpio_data[bit_start_pos] &= _colourbitsclear; // celear what was there before   

      mask = 1 << subpixel_colour_bit;      

      if (g_data & mask) {
        dma_grey_gpio_data[bit_start_pos] |= g_gpio_bitmask;
      }

      if (b_data & mask) {
        dma_grey_gpio_data[bit_start_pos] |= b_gpio_bitmask;
      }

      if (r_data & mask) {
        dma_grey_gpio_data[bit_start_pos] |= r_gpio_bitmask;
      }

      //ESP_LOGV(TAG, "Setting dma_grey_gpio_data from bit_start_pos %d. Value %d", bit_start_pos, dma_grey_gpio_data[bit_start_pos]);
      bit_start_pos++;
    }

    // We assumpt bit_start_postions that should be cleared, are.
        
 
  }  // mbi_set_pixel


  /* Pre-activation command - sent before sending configuration register data */
  void mbi_pre_active_dma() {
    ESP_LOGD(TAG, "Send MBI Pre-Active.");

    int payload_length = 0;  // length in parallel CLOCK cycles
    for (int i = 0; i < 14; i++) {
      dma_grey_gpio_data[payload_length] = BIT_LAT;
      payload_length++;
    }

    /* LE/LAT should be low for any rising edge of DCLK */
    for (int i = 0; i < 2; i++) {
      dma_grey_gpio_data[payload_length] = 0x00;
      payload_length++;
    }

    dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

    /*
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      mbi_clock(14);
      gpio_set_level((gpio_num_t)MBI_LAT, 0);
      mbi_clock(2);
    */
  }  // mbi_pre_active_dma

  /* Vertical Sync - updates frame data on outputs, used in conjunction with vertical scan 

    9. GCLK must keep at low level more than 7ns before MBI5152 receives the Vsync signa
    7ns = 142 Mhz clock speed...
    
    After the last data latch command, it needs at least 50 GCLKs to read the gray scale data into internal display
    buffer before the Vsync command comes. And display is updated immediately until MBI5051/52/53 receives
    the Vsync signal (high pulse of LE pin is sampled by 3-DCLK rising edges), as figure 6 shows.
  */
  void mbi_v_sync_dma() {
    ESP_LOGV(TAG, "Send MBI Vert Sync.");
  
    /*
    int payload_length = 0;
    for (int i = 0; i < 3; i++) {
      dma_grey_gpio_data[payload_length] = BIT_LAT;
      payload_length++;
    }

    dma_grey_gpio_data[payload_length] = 0x00;
    payload_length++;
    */

    // Send the Vsync somewhere in the middle of the gclk data.
    int payload_length = 600; 
    memset(dma_grey_gpio_data, 0, payload_length * sizeof(ESP32_GREY_DMA_STORAGE_TYPE));

    int start_pos = payload_length - (payload_length/2); 
    for (int i = 0; i < 3; i++) {
      dma_grey_gpio_data[start_pos++] = BIT_LAT;
    }

    dma_grey_gpio_data[start_pos++] = 0x00;

    dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

    /*
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      mbi_clock(2);
      gpio_set_level((gpio_num_t)MBI_LAT, 0);
    */
  }  // mbi_v_sync_dma

  void mbi_soft_reset_dma() {
    /* “Software reset” command makes MBI5153 go back to the initial state except configuration register value. After this
    command is received, the output channels will be turned off and will display again with last gray-scale value after
    new “Vsync” command is received.
    */
    ESP_LOGD(TAG, "Send MBI Soft Reset.");

    int payload_length = 0;
    for (int i = 0; i < 10; i++) {
      dma_grey_gpio_data[payload_length] = BIT_LAT;
      payload_length++;
    }

    dma_grey_gpio_data[payload_length] = 0x00;
    payload_length++;

    dma_bus.send_stuff_once(dma_grey_gpio_data, payload_length * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);

    /*
      gpio_set_level((gpio_num_t)MBI_LAT, 1);
      mbi_clock(10);
      gpio_set_level((gpio_num_t)MBI_LAT, 0);
    */
  }  // mbi_soft_reset_dma

  /* Need to set config for all MBI input on the panel at once as they all share the same latch pin */
  void mbi_set_config_dma(unsigned int &dma_output_pos, uint16_t config_reg, bool latch, bool reg2) {
    ESP_LOGD(TAG, "Send MBI config");

    // Number of DCLK Rising Edge when LE is asserted
    // Write Configuration 1 = 4
    // Write Configuration 2 = 8
    int latch_trigger_point = (reg2) ? 8 : 4;

    for (int bit = 15; bit >= 0; bit--) {
      int bitval = ((config_reg >> bit) & 1);

      uint16_t mbi_rgb_sdi_val = 0;
      if (bitval) {
        mbi_rgb_sdi_val = BIT_ALL_RGB;  // all <BI colour channels get the same config for now.
      }

      if ((bit < latch_trigger_point) && (latch == true)) {  // for reg1, data latch on the last 4 bits when latch is true (on last send)

        // std::cout << "Latch." << std::endl;
        mbi_rgb_sdi_val |= BIT_LAT;
      }

      dma_grey_gpio_data[dma_output_pos++] = mbi_rgb_sdi_val;

    }  // iterate through bits

  }  // mbi_send_config_dma

  /* Configure main register */
  void mbi_send_config_reg1_dma() {
    static int config_payload_bit_pos = 0;
    // Step 1) Send configuration for
    uint16_t config_reg1_val = 0;

    int ghost_elimination = ghost_elimination_ON;
    int line_num = PANEL_SCAN_LINES - 1;
    int gray_scale = gray_scale_14;
    int gclk_multiplier = gclk_multiplier_OFF;
    int current = current_1;  // change as required by channel

    // Documentation says set bits E and F of Config1 Reg to 1
    config_reg1_val = (config_reg1_val | (ghost_elimination << 14) | (line_num << 8) | (gray_scale << 7) | (gclk_multiplier << 6) | (current));

    unsigned int dma_output_pos = 0;
    for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
      mbi_set_config_dma(dma_output_pos, config_reg1_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), false);  // on last mbi chip, latch
    }

    dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);
  }  // mbi_send_config_reg1_dma

  /*Ебать Россию, ебать Путина*/
  void mbi_send_config_reg2_dma() {
    // F E D C B A 9 8 7 6 5 4 3 2 1 0
    // uint16_t config_reg2_val = 0b0001000000010000; // default value apparently

    uint16_t config_reg2_val = 0b1001000000011110;  // removes ghosting except for red
    unsigned int dma_output_pos = 0;

    for (int i = 0; i < PANEL_MBI_CHAIN_LEN; i++) {
      mbi_set_config_dma(dma_output_pos, config_reg2_val, (i == (PANEL_MBI_CHAIN_LEN - 1)), true);  // on last mbi chip, latch
    }

    dma_bus.send_stuff_once(dma_grey_gpio_data, dma_output_pos * sizeof(ESP32_GREY_DMA_STORAGE_TYPE), false);
  }  // mbi_send_config_reg2_dma
};

#endif  // MATRIX_H
