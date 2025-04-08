/******************************************************************************************
 * @file        main.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S2 implementation for a MBI5135 PWM chip based LED Matrix Panel
 * 
 *  Mr...
 *   ██████╗ ██████╗ ██████╗ ███████╗████████╗ █████╗ ███████╗████████╗██╗ ██████╗
 *  ██╔════╝██╔═══██╗██╔══██╗██╔════╝╚══██╔══╝██╔══██╗██╔════╝╚══██╔══╝██║██╔════╝
 *  ██║     ██║   ██║██║  ██║█████╗     ██║   ███████║███████╗   ██║   ██║██║     
 *  ██║     ██║   ██║██║  ██║██╔══╝     ██║   ██╔══██║╚════██║   ██║   ██║██║     
 *  ╚██████╗╚██████╔╝██████╔╝███████╗   ██║   ██║  ██║███████║   ██║   ██║╚██████╗
 *    ╚═════╝ ╚═════╝ ╚═════╝ ╚══════╝   ╚═╝   ╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝ ╚═════╝
 *                                                                            .... was here
 ******************************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "rom/cache.h"

#include "esp_timer.h"
#include "rtc.h"
#include "esp32s2/rom/rtc.h"

#include <cmath>

#include "esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp"
#include "app_constants.h"  
#include "GFX_Layer.hpp"

#include "Fonts/FreeSansBold9pt7b.h" // include adafruit font

/***************************************************************/

const gpio_num_t ledPin = GPIO_NUM_15;  // LED pin

/***************************************************************/

// Global GFX_Layer object
GFX_Layer gfx_layer_bg(PANEL_PHY_RES_X, PANEL_PHY_RES_Y, mbi_set_pixel); // background
GFX_Layer gfx_layer_fg(PANEL_PHY_RES_X, PANEL_PHY_RES_Y, mbi_set_pixel); // foreground

GFX_LayerCompositor gfx_compositor(mbi_set_pixel);

/***************************************************************/
static uint8_t s_led_state = 0;

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;
TaskHandle_t myTaskhandle3 = NULL;


static float angle = 0.0f;
static uint16_t r,g,b;

static void blink_led(void) {
    s_led_state = !s_led_state;
    gpio_set_level(ledPin, s_led_state);
}

void output_task(void *arg) {
    while (1) {

      blink_led();
      printf("Free internal SRAM heap available: %zu bytes\n", heap_caps_get_free_size( MALLOC_CAP_INTERNAL ));
      printf("I2S DMA out_eof interrupt count is: %d\n", get_interrupt_count());

 
      // If you don't have the delay, the main core thead will never get time again.
      for (int i = 5; i >= 0; i--) { // wait 10 seconds
          vTaskDelay(1000 / portTICK_PERIOD_MS);
      }

      fflush(stdout);
    }
} // end output_task


/************************************************************************/

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

float tx, nx, p;
float ty, ny, py;
float rot, rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
int i; //0 to 360
int fl, scale; //focal length
int wireframe[12][2];

int originx = 40;
int originy = 40; //32

int front_depth = 20;
int back_depth = -20;

//Store cube vertices
int cube_vertex[8][3] = {
 { -20, -20, front_depth},
 {20, -20, front_depth},
 {20, 20, front_depth},
 { -20, 20, front_depth},
 { -20, -20, back_depth},
 {20, -20, back_depth},
 {20, 20, back_depth},
 { -20, 20, back_depth}
};

int fd = 0; //0=orthographic


void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1  ) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {

      gfx_layer_bg.setPixel(y0, x0, 0,255,255);
    } else {
      gfx_layer_bg.setPixel(x0, y0, 255,0,255);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void draw_vertices(void)
{
   gfx_layer_bg.setPixel(rotxxx, rotyyy, 255, 0, 0);
}

void draw_wireframe(void)
{
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[1][0], wireframe[1][1]);
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[2][0], wireframe[2][1]);
 writeLine(wireframe[2][0], wireframe[2][1], wireframe[3][0], wireframe[3][1]);
 writeLine(wireframe[3][0], wireframe[3][1], wireframe[0][0], wireframe[0][1]);

//cross face above
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[3][0], wireframe[3][1]);
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[2][0], wireframe[2][1]);

 writeLine(wireframe[4][0], wireframe[4][1], wireframe[5][0], wireframe[5][1]);
 writeLine(wireframe[5][0], wireframe[5][1], wireframe[6][0], wireframe[6][1]);
 writeLine(wireframe[6][0], wireframe[6][1], wireframe[7][0], wireframe[7][1]);
 writeLine(wireframe[7][0], wireframe[7][1], wireframe[4][0], wireframe[4][1]);
 
 writeLine(wireframe[0][0], wireframe[0][1], wireframe[4][0], wireframe[4][1]);
 writeLine(wireframe[1][0], wireframe[1][1], wireframe[5][0], wireframe[5][1]);
 writeLine(wireframe[2][0], wireframe[2][1], wireframe[6][0], wireframe[6][1]);
 writeLine(wireframe[3][0], wireframe[3][1], wireframe[7][0], wireframe[7][1]);
}


unsigned long lastTime = 0;
unsigned long currentTime = 0;
int frame_count = 0;

void spinning_cube_task(void *arg) {

    gfx_layer_fg.clear();            
    gfx_layer_fg.drawCentreText("COOOL!", MIDDLE, &FreeSansBold9pt7b, CRGB(0, 0, 255));
    gfx_layer_fg.autoCenterX(); // because I don't trust AdaFruit to perfectly place the contents in the middle


    while (1) {
      
      currentTime = millis();

      if ((currentTime - lastTime) > 1000)
      {
          printf("Frame Rate is: %d\n", frame_count);

          frame_count = 0;
          lastTime = currentTime;

      }       

     for (int angle = 0; angle <= 360; angle = angle + 3) {
        for (int i = 0; i < 8; i++) {

          rot = angle * 0.0174532; //0.0174532 = one degree
          //rotateY
          rotz = cube_vertex[i][2] * cos(rot) - cube_vertex[i][0] * sin(rot);
          rotx = cube_vertex[i][2] * sin(rot) + cube_vertex[i][0] * cos(rot);
          roty = cube_vertex[i][1];
          //rotateX
          rotyy = roty * cos(rot) - rotz * sin(rot);
          rotzz = roty * sin(rot) + rotz * cos(rot);
          rotxx = rotx;
          //rotateZ
          rotxxx = rotxx * cos(rot) - rotyy * sin(rot);
          rotyyy = rotxx * sin(rot) + rotyy * cos(rot);
          rotzzz = rotzz;

          //orthographic projection
          rotxxx = rotxxx + originx;
          rotyyy = rotyyy + originy;

          //store new vertices values for wireframe drawing
          wireframe[i][0] = rotxxx;
          wireframe[i][1] = rotyyy;
          wireframe[i][2] = rotzzz;
          draw_vertices();
       }


        frame_count++;    
        gfx_layer_bg.dim(180); // dim existing pixels
        draw_wireframe(); // draw to GFX Layer BUFFER!

        delay(25);
        
        // this will send to panel output as well via. callback
        gfx_compositor.Blend(gfx_layer_bg, gfx_layer_fg); 
 
        //mbi_clear(); // don't need this if using layers       
    } // draw wireframe     
       

  } // end while

} // end cube task


/************************************************************************/
void rgb_loop_task(void *arg) {

    static int rgb_mode = 0;

    while (1) {
 

      int r = 0;
      int g = 0;
      int b = 0;
      if (rgb_mode == 0 )
      {
        r = 255;
        rgb_mode++;        
      }
      else if (rgb_mode == 1)
      {
        g = 255;
        rgb_mode++;        
      }
      else if (rgb_mode == 2)
      {
        b = 255;
        rgb_mode++;
      }
     else if (rgb_mode == 3) // black
      {
        rgb_mode = 0;
      }      

      for (int y = 0; y < PANEL_MBI_RES_Y; y++) {
              for (int x = 0; x < PANEL_MBI_RES_X; x++) {                
                  mbi_set_pixel(x, y, r,g,b);
              }
          }

        mbi_update();
        vTaskDelay(1000 / portTICK_PERIOD_MS);              
        //mbi_clear();  

    } // draw wireframe     
       
} // end cube task

/************************************************************************/



void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    
    // Setup GPIO Pin
    gpio_reset_pin(ledPin);
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);

    blink_led();    

    for (int delaysec = 3; delaysec > 0; delaysec--)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP_LOGI("app_main", "Starting in %d...", delaysec);
    }

    ESP_LOGI("app_main", "Calling mbi_start()");
    mbi_start();

    ESP_LOGI("app_main", "Creating task to do regular update on status etc.");        
    xTaskCreate(output_task, "Output_Task", 4096, NULL, 10, &myTaskHandle1);


    ESP_LOGI("app_main", "Creating graphics task.");        
    xTaskCreate(spinning_cube_task, "GraphicsLayer_Task", 4096, NULL, 10, &myTaskHandle2);

    //xTaskCreate(rgb_loop_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);

}

void loop()
{
    vTaskDelete(NULL); // Delete Loop task, we don't need it
}