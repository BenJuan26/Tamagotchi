/*
 * ArduinoGotchi - A real Tamagotchi emulator for Arduino UNO
 *
 * Copyright (C) 2022 Gary Kwok
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <Arduino.h>
#include <Wire.h>

#include "tamalib.h"
#include "hw.h"
#include "bitmaps.h"
#if defined(ENABLE_AUTO_SAVE_STATUS) || defined(ENABLE_LOAD_STATE_FROM_EEPROM)
#include "savestate.h"
#endif

#if defined(ENABLE_OTA) || defined(WEBSERIAL)
#include "wifi_creds.h"
#include <WiFi.h>
#endif

#if defined(ENABLE_OTA)
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

#if defined(WEBSERIAL)
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80);
#endif

void print(const char *msg) {
  Serial.print(msg);
#if defined(WEBSERIAL)
  WebSerial.print(msg);
#endif
}

void println(const char *msg) {
  Serial.println(msg);
#if defined(WEBSERIAL)
  WebSerial.println(msg);
#endif
}

const uint8_t iconPins[ICON_NUM] = {5, 15, 32, 33, 25, 26, 27, 21};

#if defined(USE_PX_MATRIX)
#include <PxMatrix.h>
#include <Ticker.h>
Ticker display_ticker;

// see readme https://github.com/2dom/PxMatrix
#define A 19
#define B 23
#define C 18
#define LAT 22
#define P_OE 16

#if defined(ESP32)
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// #define PxMATRIX_double_buffer true

// This defines the 'on' time of the display is us. The larger this number,
// the brighter the display. If too large the ESP will crash
uint8_t display_draw_time=70; //30-70 is usually fine

PxMATRIX matrix(32, 16, LAT, P_OE, A, B, C);
const uint8_t bg_data[] = {12, 132, 28, 10, 6, 70, 248, 10, 6, 99, 241, 234, 6, 96, 128, 116, 6, 96, 254, 84, 12, 64, 14, 84, 24, 192, 116, 172, 33, 193, 128, 172, 97, 134, 1, 172, 225, 206, 51, 44, 224, 232, 31, 38, 240, 25, 206, 54, 124, 0, 124, 179, 63, 0, 49, 155, 15, 248, 113, 153, 0, 15, 159, 25};
const uint16_t bg_green_base = matrix.color565(181, 184, 97);
const uint16_t bg_blue_base = matrix.color565(105, 169, 167);

#if defined(ESP32)
void IRAM_ATTR display_updater(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  matrix.display(display_draw_time);
  portEXIT_CRITICAL_ISR(&timerMux);
}
#else
// ISR for display refresh
void display_updater()
{
  matrix.display(display_draw_time);
}
#endif

#if defined(ESP32)
void display_update_enable(bool is_enable)
{
  if (is_enable)
  {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &display_updater, true);
    timerAlarmWrite(timer, 4000, true);
    timerAlarmEnable(timer);
  }
  else
  {
    timerDetachInterrupt(timer);
    timerAlarmDisable(timer);
  }
}
#else
void display_update_enable(bool is_enable)
{
  if (is_enable)
    display_ticker.attach(0.004, display_updater);
  else
    display_ticker.detach();
}
#endif

#else
#include <RGBmatrixPanel.h>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  8   // USE THIS ON ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2
#define D   A3

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false);
const uint16_t color = matrix.Color888(127, 0, 0); // red, medium-brightness
#endif

#define PRESSED HIGH

#if defined(ESP8266_KIT_A)
#define PIN_BTN_L D3
#define PIN_BTN_M 7
#define PIN_BTN_R 13
#define PIN_BUZZER 2
#elif defined(ESP8266_KIT_B)
#define PIN_BTN_L 12
#define PIN_BTN_M 13
#define PIN_BTN_R 15
#define PIN_BUZZER 0
#define ENABLE_TAMA_SOUND
#define ENABLE_TAMA_SOUND_ACTIVE_LOW
#elif defined(ESP32)
#define PIN_BTN_L 4
#define PIN_BTN_M 0
#define PIN_BTN_R 2
#define PIN_BUZZER 255
#undef PRESSED
#define PRESSED LOW
#else
#define PIN_BTN_L 2
#define PIN_BTN_M 3
#define PIN_BTN_R 4
#define PIN_BUZZER 9
#endif

void displayTama();

/**** TamaLib Specific Variables ****/
static uint16_t current_freq = 0;
static bool_t matrix_buffer[LCD_HEIGHT][LCD_WIDTH / 8] = {{0}};
//static byte runOnceBool = 0;
static cpu_state_t cpuState;
static unsigned long lastSaveTimestamp = 0;
/************************************/

static void hal_halt(void)
{
  // Serial.println("Halt!");
}

static void hal_log(log_level_t level, char *buff, ...)
{
  Serial.println(buff);
}

static timestamp_t hal_get_timestamp(void)
{
#if defined(ESP32)
  return esp_timer_get_time();
#else
  return millis() * 1000;
#endif
}

static void hal_sleep_until(timestamp_t ts)
{
  // int32_t remaining = (int32_t) (ts - hal_get_timestamp());
  // if (remaining > 0) {
  //   delayMicroseconds(remaining);
  // }
}

static void hal_update_screen(void)
{
  displayTama();
}

static void hal_set_lcd_matrix(u8_t x, u8_t y, bool_t val)
{
  uint8_t mask;
  if (val)
  {
    mask = 0b10000000 >> (x % 8);
    matrix_buffer[y][x / 8] = matrix_buffer[y][x / 8] | mask;
  }
  else
  {
    mask = 0b01111111;
    for (byte i = 0; i < (x % 8); i++)
    {
      mask = (mask >> 1) | 0b10000000;
    }
    matrix_buffer[y][x / 8] = matrix_buffer[y][x / 8] & mask;
  }
}

static void hal_set_lcd_icon(u8_t icon, bool_t val)
{
  int ledState = LOW;
  if (val) {
    ledState = HIGH;
  }

  digitalWrite(iconPins[icon], ledState);
}

static void hal_set_frequency(u32_t freq)
{
  current_freq = freq;
}

static void hal_play_frequency(bool_t en)
{
#ifdef ENABLE_TAMA_SOUND
  if (en)
  {
    tone(PIN_BUZZER, current_freq);
  }
  else
  {
    noTone(PIN_BUZZER);
    #ifdef ENABLE_TAMA_SOUND_ACTIVE_LOW
    digitalWrite(PIN_BUZZER, HIGH);
    #endif
  }
#endif
}

// static bool_t button4state = 0;

static int hal_handler(void)
{
#ifdef ENABLE_SERIAL_DEBUG_INPUT
  if (Serial.available() > 0)
  {
    int incomingByte = Serial.read();
    Serial.println(incomingByte, DEC);
    if (incomingByte == 49)
    {
      hw_set_button(BTN_LEFT, BTN_STATE_PRESSED);
    }
    else if (incomingByte == 50)
    {
      hw_set_button(BTN_LEFT, BTN_STATE_RELEASED);
    }
    else if (incomingByte == 51)
    {
      hw_set_button(BTN_MIDDLE, BTN_STATE_PRESSED);
    }
    else if (incomingByte == 52)
    {
      hw_set_button(BTN_MIDDLE, BTN_STATE_RELEASED);
    }
    else if (incomingByte == 53)
    {
      hw_set_button(BTN_RIGHT, BTN_STATE_PRESSED);
    }
    else if (incomingByte == 54)
    {
      hw_set_button(BTN_RIGHT, BTN_STATE_RELEASED);
    }
  }
#else
  if (digitalRead(PIN_BTN_L) == PRESSED)
  {
    hw_set_button(BTN_LEFT, BTN_STATE_PRESSED);
  }
  else
  {
    hw_set_button(BTN_LEFT, BTN_STATE_RELEASED);
  }
  if (digitalRead(PIN_BTN_M) == PRESSED)
  {
    hw_set_button(BTN_MIDDLE, BTN_STATE_PRESSED);
  }
  else
  {
    hw_set_button(BTN_MIDDLE, BTN_STATE_RELEASED);
  }
  if (digitalRead(PIN_BTN_R) == PRESSED)
  {
    hw_set_button(BTN_RIGHT, BTN_STATE_PRESSED);
  }
  else
  {
    hw_set_button(BTN_RIGHT, BTN_STATE_RELEASED);
  }
// #ifdef ENABLE_AUTO_SAVE_STATUS
//   if (digitalRead(PIN_BTN_SAVE) == HIGH)
//   {
//     if (button4state == 0)
//     {
//       saveStateToEEPROM(&cpuState);
//     }
//     button4state = 1;
//   }
//   else
//   {
//     button4state = 0;
//   }
// #endif
#endif
  return 0;
}

static hal_t hal = {
    .halt = &hal_halt,
    .log = &hal_log,
    .sleep_until = &hal_sleep_until,
    .get_timestamp = &hal_get_timestamp,
    .update_screen = &hal_update_screen,
    .set_lcd_matrix = &hal_set_lcd_matrix,
    .set_lcd_icon = &hal_set_lcd_icon,
    .set_frequency = &hal_set_frequency,
    .play_frequency = &hal_play_frequency,
    .handler = &hal_handler,
};

void drawTamaRow(uint8_t y)
{
  uint8_t x;
  for (x = 0; x < LCD_WIDTH; x++)
  {
    uint16_t color;
    uint8_t mask = 0b10000000;
    mask = mask >> (x % 8);
    if ((matrix_buffer[y][x / 8] & mask) != 0)
    {
      color = 0;
    } else {
      uint8_t array_index = (x/8) + (y*4);
      if ((bg_data[array_index] & mask) != 0) {
        color = bg_blue_base;
      } else {
        color = bg_green_base;
      }
    }
    matrix.drawPixel(x, y, color);
  }
}

// void drawTamaSelection(uint8_t y)
// {
//   uint8_t i;
//   for (i = 0; i < 7; i++)
//   {
//     if (icon_buffer[i])
//       drawTriangle(i * 16 + 5, y);
//     display.drawXBMP(i * 16 + 4, y + 6, 16, 9, bitmaps + i * 18);
//   }
//   if (icon_buffer[7])
//   {
//     drawTriangle(7 * 16 + 5, y);
//     display.drawXBMP(7 * 16 + 4, y + 6, 16, 9, bitmaps + 7 * 18);
//   }
// }

void displayTama()
{
  for (int y = 0; y < LCD_HEIGHT; y++)
  {
    drawTamaRow(y);
  }
}

#ifdef ENABLE_DUMP_STATE_TO_SERIAL_WHEN_START
void dumpStateToSerial()
{
  uint16_t i, count = 0;
  char tmp[10];
  cpu_get_state(&cpuState);
  u4_t *memTemp = cpuState.memory;
  uint8_t *cpuS = (uint8_t *)&cpuState;

  Serial.println("");
  Serial.println("static const uint8_t hardcodedState[] PROGMEM = {");
  for (i = 0; i < sizeof(cpu_state_t); i++, count++)
  {
    sprintf(tmp, "0x%02X,", cpuS[i]);
    Serial.print(tmp);
    if ((count % 16) == 15)
      Serial.println("");
  }
  for (i = 0; i < MEMORY_SIZE; i++, count++)
  {
    sprintf(tmp, "0x%02X,", memTemp[i]);
    Serial.print(tmp);
    if ((count % 16) == 15)
      Serial.println("");
  }
  Serial.println("};");
  /*
    Serial.println("");
    Serial.println("static const uint8_t bitmaps[] PROGMEM = {");
    for(i=0;i<144;i++) {
      sprintf(tmp, "0x%02X,", bitmaps_raw[i]);
      Serial.print(tmp);
      if ((i % 18)==17) Serial.println("");
    }
    Serial.println("};");  */
}
#endif

void setup()
{
  Serial.begin(SERIAL_BAUD);

#if defined(ENABLE_OTA) || defined(WEBSERIAL)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
#endif
#if defined(ENABLE_OTA)
  ArduinoOTA.setHostname("tamagotchi");

  ArduinoOTA
    .onStart([]() {
#ifdef USE_PX_MATRIX
      display_update_enable(false);
#endif
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
#ifdef USE_PX_MATRIX
      display_update_enable(true);
#endif
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif

#if defined(WEBSERIAL)
  WebSerial.begin(&server);
  server.begin();
#endif

  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_M, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  // pinMode(PIN_BUZZER, OUTPUT);

  for (int i = 0; i < ICON_NUM; i++) {
    pinMode(iconPins[i], OUTPUT);
    digitalWrite(iconPins[i], LOW);
  }

  matrix.begin();

  tamalib_register_hal(&hal);
  tamalib_set_framerate(TAMA_DISPLAY_FRAMERATE);
  tamalib_init(1000000);

#ifdef ENABLE_LOAD_STATE_FROM_EEPROM
  initEEPROM();
  if (validEEPROM())
  {
    loadStateFromEEPROM(&cpuState);
  } else {
    println("No magic number in state, skipping state restore");
  }
#elif ENABLE_LOAD_HARCODED_STATE_WHEN_START
  initEEPROM();
  loadHardcodedState();
#endif

#ifdef ENABLE_DUMP_STATE_TO_SERIAL_WHEN_START
  dumpStateToSerial();
#endif

#ifdef USE_PX_MATRIX
  display_update_enable(true);
#endif

  println("initialized");
}

uint32_t right_long_press_started = 0;

void loop()
{
#if defined(ENABLE_OTA)
  ArduinoOTA.handle();
#endif

  tamalib_mainloop_step_by_step();
#ifdef ENABLE_AUTO_SAVE_STATUS
  if ((millis() - lastSaveTimestamp) > (AUTO_SAVE_MINUTES * 60 * 1000))
  {
    lastSaveTimestamp = millis();
#if defined(ESP32)
    display_update_enable(false);
#endif
    saveStateToEEPROM(&cpuState);
#if defined(ESP32)
    display_update_enable(true);
#endif
  }

  if (digitalRead(PIN_BTN_M) == PRESSED) {
    if (millis() - right_long_press_started > AUTO_SAVE_MINUTES * 1000) 
    {
#if defined(ESP32)
      display_update_enable(false);
#endif
      eraseStateFromEEPROM();
#if defined(ESP32)
      display_update_enable(true);
#endif
      #if defined(ESP8266) || defined(ESP32)
      ESP.restart();
      #endif
    }
  } else {
    right_long_press_started = millis();
  }
#endif
}
