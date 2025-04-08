/******************************************************************************************
 * @file        main.cpp
 * @author      github.com/mrcodetastic
 * @date        2024
 * @brief       ESP32-S3 implementation for a MBI5135 PWM chip based LED Matrix Panel
 ******************************************************************************************/


#include <Matrix.h>
#include <FastNoise.h>
#include <array>

Matrix matrix;

void hsvToRgb(float h, float s, float v, uint16_t &ret_r, uint16_t &ret_g, uint16_t &ret_b) {
    float r, g, b;
    int i = (int)(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    ret_r = (uint16_t)(r * 255);
    ret_g = (uint16_t)(g * 255);
    ret_b = (uint16_t)(b * 255);
}


// Start the App
void setup(void) 
{

  Serial.begin(115200);
  delay(100);
  Serial.println("Starting....");
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  esp_task_wdt_deinit();

  matrix.initMatrix();

  delay(10);

  matrix.drawLine(0,0,77,77,255,255,255); // this doesn't work??

  matrix.drawCircle(30,30,5,254,254,254);

  matrix.update();  

  delay(10000);



}

float angle = 0.0f;
uint16_t r,g,b;
int frame_count = 0;

void loop() 
{
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  frame_count++;

  uint16_t cr, cg, cb;

 // if ( frame_count < 5) {

    for (int y = 0; y < PANEL_MBI_RES_Y; y++) {
              for (int x = 0; x < PANEL_MBI_RES_X; x++) {
                  float dx = x - PANEL_MBI_RES_X / 2;
                  float dy = y - PANEL_MBI_RES_Y / 2;
                  float distance = sqrt(dx * dx + dy * dy);
                  float theta = atan2(dy, dx) + angle;
                  float hue = fmod((theta / (2 * PI)) + 1.0f, 1.0f);
                  hsvToRgb(hue, 1.0f, 1.0f,r,g,b);
                //  r = 255;
               //   b = 255;
               //   g = 255;
                  matrix.drawPixel(x, y, r,g,b);

                  if (y==46 & x == 39)
                  {
                    cr = r; cb = b; cg = g;
                  }

              }
    }


    matrix.fillCircleDMA(40, 40, 5, cr, cg, cb);   

    if ((currentTime - lastTime) > 1000)
    {
    //  Serial.print("FPS: ");
    //  Serial.println(frame_count, DEC);
      
    // frame_count = 0;
    //  lastTime = currentTime;

    }   

    matrix.update();
    angle += 0.01f;


 // }

}
