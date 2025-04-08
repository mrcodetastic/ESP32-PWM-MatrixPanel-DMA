/******************************************************************************************
 * @file        app_constants.h
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S2 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


#pragma once


#define GPIO_MAPPING_WEMOS_S2_MINI  1
#define GPIO_MAPPING_OTHER          2

// Selected mode
#define GPIO_MAPPING GPIO_MAPPING_WEMOS_S2_MINI


// GPIo48 is the on board RGB LED 
#if (GPIO_MAPPING == GPIO_MAPPING_WEMOS_S2_MINI)

  #define ADDR_A_PIN              GPIO_NUM_39
  #define ADDR_B_PIN              GPIO_NUM_40
  #define ADDR_C_PIN              GPIO_NUM_36
  #define ADDR_D_PIN              GPIO_NUM_33
  #define ADDR_E_PIN              GPIO_NUM_34

  #define MBI_GCLK_PIN            GPIO_NUM_38  // OE PIN IS GCLK apparently
  #define MBI_LAT_PIN             GPIO_NUM_37  //  data/command
  #define MBI_DCLK_PIN            GPIO_NUM_35  // data clocking line?

  // First 1/4 of panel -> 20 rows
  #define MBI_G1_PIN              GPIO_NUM_21 
  #define MBI_B1_PIN              GPIO_NUM_16 
  #define MBI_R1_PIN              GPIO_NUM_18  

  // Second 1/4 of panel -> 20 rows
  #define MBI_G2_PIN              GPIO_NUM_1   
  #define MBI_B2_PIN              GPIO_NUM_2  
  #define MBI_R2_PIN              GPIO_NUM_17  

  // Third 1/4 of panel -> 20 rows
  #define MBI_G3_PIN              GPIO_NUM_3  
  #define MBI_B3_PIN              GPIO_NUM_5  
  #define MBI_R3_PIN              GPIO_NUM_4  

  // Forth 1/4 of panel -> 20 rows
  #define MBI_G4_PIN              GPIO_NUM_6    
  #define MBI_B4_PIN              GPIO_NUM_8  
  #define MBI_R4_PIN              GPIO_NUM_7  

  // Debug
  #define DEBUG_PIN               GPIO_NUM_13   

#elif (GPIO_MAPPING == GPIO_MAPPING_OTHER)  


#endif


// RGB Panel Scan Lines
#define PANEL_SCAN_LINES        20 // panel scan lines / rows
#define PANEL_MBI_LED_CHANS     16  // number of led channels per MBI IC
#define PANEL_MBI_CHAIN_LEN      5  // number of ic's changed for each subpixel

#define PANEL_PHY_RES_X 78
#define PANEL_PHY_RES_Y 78

#define PANEL_MBI_RES_X 80
#define PANEL_MBI_RES_Y 80

#define ghost_elimination_ON  0b11 // послесвечение выключенно
#define ghost_elimination_OFF 0    // послесвечение включено

#define gray_scale_13         1    // шкала серого 13 бит
#define gray_scale_14         0    // шкала серого 14 бит

/* 
  GCLK multiplier (grayscale clock multiplier)

  MBI5153 provides a GCLK multiplier function by setting the configuration register1 bit [6]. 
  The default value is set to ’0’ for GCLK multiplier disable   = 513 GCLKS for each row.

  GCLK multiplier enabled (configuration register1 bit [6] = 1) = 257 GCLKS for each row!

*/

#define gclk_multiplier_ON  1  // GCLK Multipler On - You MUST use exactly 257 clocks for each rowscan!
#define gclk_multiplier_OFF 0  // GCLK Multipler On - You MUST use exactly 513 clocks for each rowscan!
#define current_1 15           // ток на светодиоде
#define current_2 63           // ток на светодиоде
#define current_3 35           // ток на светодиоде
#define current_4 20           // LED current