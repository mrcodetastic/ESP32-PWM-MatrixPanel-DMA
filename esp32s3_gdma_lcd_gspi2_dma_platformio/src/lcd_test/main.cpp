/******************************************************************************************
 * @file        lcd_test/main.cpp
 * @brief       Serial-driven LCD_CAM/GDMA and MBI command test for ESP32-S3.
 *
 * Enter one character in the serial monitor to send a single output event.
 * This keeps the LCD peripheral idle between tests, making each transfer easy
 * to inspect with a logic analyser.
 ******************************************************************************************/

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "mbi_dma_interface.h"

static MbiDmaInterface mbi;
static bool mbi_ready = false;

static void print_menu() {
  Serial.println();
  Serial.println("LCD/MBI serial test — send one character:");
  Serial.println("  r  Soft reset");
  Serial.println("  p  Pre-active command");
  Serial.println("  1  Write configuration register 1");
  Serial.println("  2  Write configuration register 2");
  Serial.println("  c  Full panel configuration sequence");
  Serial.println("  v  Vertical sync");
  Serial.println("  w  Send framebuffer with pixel (0,0) white; all others black");
  Serial.println("  h  Show this menu");
}

static void run_command(char command) {
  const char *name = nullptr;
  esp_err_t err = ESP_ERR_NOT_SUPPORTED;

  switch (command) {
    case 'r':
    case 'R':
      name = "soft reset";
      err = mbi.softReset();
      break;
    case 'p':
    case 'P':
      name = "pre-active";
      err = mbi.preActive();
      break;
    case '1':
      name = "configuration register 1";
      err = mbi.sendConfigReg1();
      break;
    case '2':
      name = "configuration register 2";
      err = mbi.sendConfigReg2();
      break;
    case 'c':
    case 'C':
      name = "full panel configuration";
      err = mbi.configurePanel();
      break;
    case 'v':
    case 'V':
      name = "vertical sync";
      err = mbi.sendVsync();
      break;
    case 'w':
    case 'W':
      name = "white pixel framebuffer";
      mbi.clearFrame();
      mbi.setPixel(0, 0, 255, 255, 255);
      err = mbi.sendFrame();
      break;
    case 'h':
    case 'H':
    case '?':
      print_menu();
      return;
    default:
      Serial.printf("Unknown option '%c'. Send h for the command list.\n", command);
      return;
  }

  if (err == ESP_OK) {
    Serial.printf("Sent %s (DMA transfers: %d).\n", name, mbi.getTransferCount());
  } else {
    Serial.printf("%s failed: %s (%d)\n", name, esp_err_to_name(err), (int)err);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  esp_task_wdt_deinit();

  Serial.println("LCD/MBI peripheral test starting...");
 
  const esp_err_t err = mbi.begin();
  if (err != ESP_OK) {
    Serial.printf("MbiDmaInterface::begin() failed: %s (%d)\n",
                  esp_err_to_name(err), (int)err);
  } else {
    mbi_ready = true;
    Serial.println("LCD_CAM/GDMA ready; no output is sent until a command is selected.");
  }

  print_menu();
}

void loop() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    if (command == '\r' || command == '\n') {
      continue;
    }
    if (!mbi_ready) {
      Serial.println("MBI interface is not ready; reset or correct the startup error first.");
      continue;
    }
    run_command(command);
  }
  delay(5);
}
