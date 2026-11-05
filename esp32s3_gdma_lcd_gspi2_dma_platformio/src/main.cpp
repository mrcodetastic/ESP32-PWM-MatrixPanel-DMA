/******************************************************************************************
 * @file        main.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S3 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


#include <Matrix.h>
#include <array>
#include <esp_system.h>
#include <esp_timer.h>

Matrix matrix;

// Start the App
void setup(void)
{

  Serial.begin(115200);
 // Serial.setDebugOutput(true); // Enable debug output for Serial to CDC interface

  delay(100);
  Serial.println("Starting....");

  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  delay(100); // let cdc get setup

  matrix.start();

  delay(100);

  matrix.flushFramebuffer();
  matrix.drawLine(0, 0, PANEL_PHY_RES_X - 1, PANEL_PHY_RES_Y - 1, RGB(255, 255, 255));
  matrix.drawLine(PANEL_PHY_RES_X - 1, 0, 0, PANEL_PHY_RES_Y - 1, RGB(255, 255, 255));  
  matrix.update();
  delay(1000);

  matrix.fillScreen(RGB(255, 0, 0));
  matrix.update();  
  delay(1000);

  matrix.fillScreen(RGB(0, 255, 0));
  matrix.update();
  delay(1000);

  matrix.fillScreen(RGB(0, 0, 255));
  matrix.update();  
  delay(1000);  
  
}

void loop() {
#if defined(MATRIX_PROFILE)
  const int64_t loop_start_us = esp_timer_get_time();
  static uint32_t profile_frame_count = 0;
  static int64_t profile_render_time_us = 0;
  static int64_t profile_loop_time_us = 0;
#endif
  // Canvas loop
  float t = (float) ((millis()%6000)/6000.f);
  float tt = (float) ((millis()%28000)/28000.f);

  for(int x = 0; x < (PANEL_MBI_RES_X); x++)
  {
    // calculate the overal shade
    float f = (((sin(tt-(float)x/PANEL_MBI_RES_Y/32.)*2.f*PI)+1)/2)*255;
    // calculate hue spectrum into rgb
    float r = max(min(cosf(2.f*PI*(t+((float)x/PANEL_MBI_RES_Y+0.f)/3.f))+0.5f,1.f),0.f);
    float g = max(min(cosf(2.f*PI*(t+((float)x/PANEL_MBI_RES_Y+1.f)/3.f))+0.5f,1.f),0.f);
    float b = max(min(cosf(2.f*PI*(t+((float)x/PANEL_MBI_RES_Y+2.f)/3.f))+0.5f,1.f),0.f);

    // iterate pixels for every row
    for(int y = 0; y < PANEL_MBI_RES_Y; y++){
      if(y*2 < PANEL_MBI_RES_Y){
        // top-middle part of screen, transition of value
        float t = (2.f*y+1)/PANEL_MBI_RES_Y;
        matrix.drawPixel(x,y,
          (r*t)*f,
          (g*t)*f,
          (b*t)*f
        );
      }else{
        // middle to bottom of screen, transition of saturation
        float t = (2.f*(PANEL_MBI_RES_Y-y)-1)/PANEL_MBI_RES_Y;
        matrix.drawPixel(x,y,
          (r*t+1-t)*f,
          (g*t+1-t)*f,
          (b*t+1-t)*f
        );
      }
    }
  }

    matrix.drawPixel(2, 2, RGB(0, 0, 255));    
#if defined(MATRIX_PROFILE)
    const int64_t render_done_us = esp_timer_get_time();
#endif
    matrix.update();  
#if defined(MATRIX_PROFILE)
    const int64_t loop_done_us = esp_timer_get_time();
    profile_render_time_us += render_done_us - loop_start_us;
    profile_loop_time_us += loop_done_us - loop_start_us;
    ++profile_frame_count;

    constexpr int64_t PROFILE_INTERVAL_US = 5000000;
    if (profile_loop_time_us >= PROFILE_INTERVAL_US) {
      const uint32_t frames = profile_frame_count;
      const int64_t render_time_us = profile_render_time_us;
      const int64_t loop_time_us = profile_loop_time_us;

      // Start the next measurement window before logging so serial time is excluded.
      profile_frame_count = 0;
      profile_render_time_us = 0;
      profile_loop_time_us = 0;

      const double render_fps =
          render_time_us > 0 ? frames * 1000000.0 / render_time_us : 0.0;
      const double loop_fps =
          loop_time_us > 0 ? frames * 1000000.0 / loop_time_us : 0.0;
      ESP_LOGW("main", "profile (%u frames): render=%.2f FPS, render+update=%.2f FPS",
               (unsigned)frames, render_fps, loop_fps);
    }
#endif
  
}
