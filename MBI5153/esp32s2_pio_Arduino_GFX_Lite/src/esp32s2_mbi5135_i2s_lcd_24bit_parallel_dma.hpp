/******************************************************************************************
 * @file        esp32s2_i2s_lcd_24bit_parallel_dma.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S2 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


#pragma once

#include <string.h> // memcpy
#include <stdbool.h>
#include <sys/types.h>
#include <freertos/FreeRTOS.h>
//#include <driver/i2s.h>
#include <rom/lldesc.h>
#include <rom/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <driver/i2s_types.h> //includes struct and reg
#else
#include <driver/i2s.h>
#include <soc/i2s_struct.h>
#endif

#include <soc/i2s_periph.h> //includes struct and reg

#define DMA_MAX (4096-4)

// The type used for this SoC
#define HUB75_DMA_DESCRIPTOR_T lldesc_t


/*

+--------------+--------------------+--------------------+--------------------+--------------------+
| gpio  byte   | 24bit output clk 1 | 24bit output clk 2 | 24bit output clk 3 | 24bit output clk 4 |
+--------------+--------------------+--------------------+--------------------+--------------------+
| d7-d0 byte   | 0                  | 3                  | 6                  | 9                  |
| d15-d8 byte  | 1                  | 4                  | 7                  | 10                 |
| d23-d16 byte | 2                  | 5                  | 8                  | 11                 |
+--------------+--------------------+--------------------+--------------------+--------------------+

*/

// Ensure this layout matches that of the GPIO bit output mapping order.
typedef struct {

  //uint8_t byte0; // lsb
  union {
    struct {
        uint8_t a:1; //lsb
        uint8_t b:1;
        uint8_t c:1;
        uint8_t d:1;
        uint8_t e:1;
        uint8_t notused0:1;
        uint8_t lat:1;
        uint8_t gclk:1;
    };
    uint8_t byte0;
  };
   union {
        struct {
            uint8_t r1:1;
            uint8_t g1:1;
            uint8_t b1:1;
            uint8_t r2:1;
            uint8_t g2:1;
            uint8_t b2:1;
            uint8_t notused1:1; // not used anymore, use LCD periph clock
            uint8_t notused2:1;            
        };
        uint8_t byte1;
    };
    union {
        struct {
            uint8_t r3:1;
            uint8_t g3:1;
            uint8_t b3:1;
            uint8_t r4:1;
            uint8_t g4:1;
            uint8_t b4:1;
            uint8_t notused3:1;
            uint8_t notused4:1;
        };
        uint8_t byte2;
    };

} out24v2;


//#define DMA_DATA_TYPE out24

//#define DMA_DATA_TYPE uint8_t

#define DMA_DATA_TYPE out24v2

//----------------------------------------------------------------------------

 struct config_t
  {
    // Default values
    uint32_t bus_freq;
    int8_t pin_wr; 
    bool   invert_pclk;
    int8_t parallel_width;

    union
    {
      int8_t pin_data[24];
      struct
      {
        int8_t pin_d0;
        int8_t pin_d1;
        int8_t pin_d2;
        int8_t pin_d3;
        int8_t pin_d4;
        int8_t pin_d5;
        int8_t pin_d6;
        int8_t pin_d7;
        int8_t pin_d8;
        int8_t pin_d9;
        int8_t pin_d10;
        int8_t pin_d11;
        int8_t pin_d12;
        int8_t pin_d13;
        int8_t pin_d14;
        int8_t pin_d15;
        int8_t pin_d16;          
        int8_t pin_d17;               
        int8_t pin_d18;                         
        int8_t pin_d19;                                   
        int8_t pin_d20;                                             
        int8_t pin_d21;                                                       
        int8_t pin_d22;                                                                 
        int8_t pin_d23;                                                                           
      };
    };
  };

//----------------------------------------------------------------------------
 
    int       get_interrupt_count();

    esp_err_t mbi_start(); // does everything
    
    void mbi_clear();    
    void mbi_set_pixel(int16_t x, int16_t y, uint8_t r_data, uint8_t g_data, uint8_t b_data);
    void mbi_update();    

    void dma_set_vsync();
    void dma_clr_vsync();
