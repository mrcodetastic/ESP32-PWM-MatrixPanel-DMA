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

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "esp32s2_mbi5135_i2s_lcd_24bit_parallel_dma.hpp"
#include "app_constants.h"

/***************************************************************/

const gpio_num_t ledPin = GPIO_NUM_15;  // LED pin

/***************************************************************/
// Simple software framebuffer, flushed straight to the panel via mbi_set_pixel()
/***************************************************************/

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};


#define FRAMEBUFFER_PIXELS (PANEL_PHY_RES_X * PANEL_PHY_RES_Y)
#define FRAMEBUFFER_BYTES  (FRAMEBUFFER_PIXELS * sizeof(RGB))

#ifdef RGB_FRAMEBUFFER_IN_PSRAM
static RGB (*framebuffer)[PANEL_PHY_RES_X] = nullptr;
#else
static RGB framebuffer[PANEL_PHY_RES_Y][PANEL_PHY_RES_X];
#endif

static uint8_t s_led_state = 0;

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;

static void blink_led(void) {
    s_led_state = !s_led_state;
    gpio_set_level(ledPin, s_led_state);
}

/***************************************************************/
// Framebuffer helpers
/***************************************************************/

static inline void fb_clear(void) {
    memset(framebuffer, 0, FRAMEBUFFER_BYTES);
}

// Allocate/initialise the framebuffer according to RGB_FRAMEBUFFER_IN_PSRAM.
static bool fb_init(void) {
#ifdef RGB_FRAMEBUFFER_IN_PSRAM
    bool in_psram = true;
    framebuffer = static_cast<RGB (*)[PANEL_PHY_RES_X]>(
        heap_caps_malloc(FRAMEBUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (framebuffer == nullptr) {
        ESP_LOGW("app_main", "PSRAM framebuffer allocation failed; falling back to internal SRAM");
        in_psram = false;
        framebuffer = static_cast<RGB (*)[PANEL_PHY_RES_X]>(
            heap_caps_malloc(FRAMEBUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (framebuffer == nullptr) {
        ESP_LOGE("app_main", "Framebuffer allocation failed (%u bytes)", (unsigned)FRAMEBUFFER_BYTES);
        return false;
    }
    ESP_LOGI("app_main", "Framebuffer allocated in %s (%u bytes)",
             in_psram ? "PSRAM" : "internal SRAM", (unsigned)FRAMEBUFFER_BYTES);
#else
    ESP_LOGI("app_main", "Framebuffer is static in internal SRAM (%u bytes)", (unsigned)FRAMEBUFFER_BYTES);
#endif
    fb_clear();
    return true;
}

static inline void fb_set_pixel(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || y < 0 || x >= PANEL_PHY_RES_X || y >= PANEL_PHY_RES_Y) {
        return;
    }
    framebuffer[y][x].r = r;
    framebuffer[y][x].g = g;
    framebuffer[y][x].b = b;
}

// Push the whole framebuffer to the MBI panel buffer
static void fb_flush(void) {
    for (int y = 0; y < PANEL_PHY_RES_Y; y++) {
        for (int x = 0; x < PANEL_PHY_RES_X; x++) {
            const RGB &p = framebuffer[y][x];
            mbi_set_pixel(x, y, p.r, p.g, p.b);
        }
    }
    // Flush the whole greyscale region to PSRAM once, instead of per pixel.
    mbi_cache_writeback();
    mbi_update();
}

/***************************************************************/

void output_task(void *arg) {
    while (1) {

      blink_led();

      // NOTE: printf()/stdout is routed to UART0 (GPIO43/44) by the IDF console,
      // which is not connected to the native USB-C port on the S2 Mini.
      // Serial is USBSerial (USBCDC) because ARDUINO_USB_CDC_ON_BOOT=1, so use
      // Serial.printf() to reach the USB CDC monitor.
      // Also: CONFIG_NEWLIB_NANO_FORMAT=y, so avoid %zu - cast to unsigned and use %u.
      const unsigned sram_free   = (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      const unsigned sram_total  = (unsigned)heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
      const unsigned psram_free  = (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
      const unsigned psram_total = (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

      Serial.printf("Free internal SRAM heap available: %u bytes (of %u bytes)\n", sram_free, sram_total);
      Serial.printf("Free PSRAM heap available: %u bytes (of %u bytes)\n", psram_free, psram_total);
      Serial.printf("I2S DMA out_eof interrupt count is: %d\n", get_interrupt_count());

      // If you don't have the delay, the main core thread will never get time again.
      for (int i = 4; i >= 0; i--) { // wait 10 seconds
          vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
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
int fl, scale; //focal length
int wireframe[12][2];

int originx = 40;
int originy = 40;

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


void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
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
      fb_set_pixel(y0, x0, 0, 255, 255);
    } else {
      fb_set_pixel(x0, y0, 255, 0, 255);
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
   fb_set_pixel(rotxxx, rotyyy, 255, 0, 0);
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

    fb_clear();
    fb_flush();

    while (1) {

      currentTime = millis();

      if ((currentTime - lastTime) > 1000)
      {
          Serial.printf("Frame Rate is: %d\n", frame_count);

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
        fb_clear();        // wipe last frame
        draw_wireframe();  // draw into the software framebuffer
        fb_flush();        // send the framebuffer to the panel via mbi_set_pixel()

        delay(25);
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
                  mbi_set_pixel(x, y, r, g, b);
              }
          }

        mbi_update();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

/************************************************************************/

void setup()
{

    /* 
     * Serial.begin(115200) in setup() is redundant with ARDUINO_USB_CDC_ON_BOOT=1, the core's main.cpp already calls Serial.begin() 
     * and USB.begin() before setup() runs. Calling it again is harmless, but it is not what makes the port work.
     */
    //Serial.begin(115200); 
    Serial.setDebugOutput(true);   // optional: routes ESP_LOGx to the USB CDC

    // USB CDC enumerates asynchronously. Wait (bounded) for the host to open the
    // port so the first messages are not lost. The timeout prevents a hang when
    // no monitor is attached.
    // Doesn't achieve anything in principle, but it does give the host time to enumerate the USB device and open the port.
    uint32_t usb_wait_start = millis();
    while (!Serial && (millis() - usb_wait_start) < 2000) {
      delay(10);
    }

    // Setup GPIO Pin
    gpio_reset_pin(ledPin);
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);

    blink_led();

    for (int delaysec = 3; delaysec > 0; delaysec--)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ESP_LOGI("app_main", "Starting in %d...", delaysec);
    }

    ESP_LOGI("app_main", "Initialising framebuffer");
    if (!fb_init()) {
        ESP_LOGE("app_main", "Framebuffer init failed, halting");
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    ESP_LOGI("app_main", "Calling mbi_start()");
    mbi_start();

    ESP_LOGI("app_main", "Creating task to do regular update on status etc.");
    xTaskCreate(output_task, "Output_Task", 4096, NULL, 10, &myTaskHandle1);

    ESP_LOGI("app_main", "Creating graphics task.");
    xTaskCreate(spinning_cube_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);

    //xTaskCreate(rgb_loop_task, "Graphics_Task", 4096, NULL, 10, &myTaskHandle2);
}

void loop()
{
    vTaskDelete(NULL); // Delete Loop task, we don't need it
}
