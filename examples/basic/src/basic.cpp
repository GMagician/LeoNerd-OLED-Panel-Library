/**
 * Basic example for LeoNerd's OLED Panel Library
 *
 * Copyright (C) 2020-2021
 * Original code by Technik Gegg
 * Modified by GMagician
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 /*
  * Please notice: Even though this example has been written and compiled
  * for the Arduino Nano, it won't leave much storage left for anything else
  * than that.
  * If you really need such expensive interface, you should be probably using
  * a beefier MCU, such as the ATMEGA2560.
  */

// Basic includes
#include <Arduino.h>
#include "U8g2lib.h"
#include "LeoNerdPanel.h"

#define DISPLAY_ADDRESS     0x3C    // use the default address
#define ENCODER_ADDRESS     0x3D    // use the default address

// Fonts used in this program.
// Be aware that each font used will reduce the amount of
// available Flash memory!
#define BASE_FONT           u8g2_font_6x12_t_symbols
#define SMALL_FONT          u8g2_font_6x10_mr

// simple wrapper for determining the array elements count
#define ArraySize(arr)      (sizeof(arr) / sizeof(arr[0]))

// Runtime instances for display and encoder
LeoNerdPanel panel(ENCODER_ADDRESS);

// For the display we're using the famous U8G2 library because of the already
// integrated support for SH1106 but you're free to use any other library
// that supports this display type.
// Please notice: The rotation (U8G2_R2 - 180°) has to be set for this module!
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

// runtime variables
buttonState_t wheelBtn,
              mainBtn,
              leftBtn,
              rightBtn;
int16_t encoderPos = 0, lastEncoderPos = 0;

typedef struct {
  uint16_t frequency;
  uint8_t duration,
          pause;
} tune_t;

const tune_t tune[] PROGMEM = {
  { 1760,  90,  90 },
  { 1975,  90,  90 },
  { 2093,  90,  90 },
  { 1975,  90,  90 },
  { 1760, 200,  50 }
};



// Function used to print out formatted debug information
void debugEcho(const char* fmt, ...) {
  char tmp[60];
  va_list arguments;

  va_start(arguments, fmt);
  vsnprintf_P(tmp, ArraySize(tmp) - 1, fmt, arguments);
  va_end(arguments);
  Serial.print(F("Debug: "));
  Serial.println(tmp);
}

// Function used to scan for all available I2C/TWI devices on the bus
void scanI2CDevices() {
  Wire.begin();
  for(uint8_t i2cAdr = 1; i2cAdr < 127; ++i2cAdr) {
    // request a transmission for the device address
    Wire.beginTransmission(i2cAdr);
    // if something answered, the device is basically available
    if (!Wire.endTransmission())
      debugEcho(PSTR("I2C device found at address 0x%2x"), i2cAdr);
  }
}

// Function used to initialize the display
void setupDisplay() {
  display.begin();
  display.enableUTF8Print();
  display.clearDisplay();
}

// Function used to initialize the panel
void setupPanel() {
  panel.begin();
  debugEcho(PSTR("LeoNerd's panel version is: %d"), panel.queryVersion());    // Please notice: This library needs at least version 2!
  debugEcho(PSTR("Options set: 0x%02X"), panel.queryOptions());               // check the options
}

// Function used to translate the button states into readable text
const char* PROGMEM stringifyButtonState(buttonState_t state) {
  switch (state) {
    case Open:
      return PSTR("None");

    case Clicked:
      return PSTR("Click");

    case LongClicked:
      return PSTR("Hold");

    default:
      return PSTR("");
  }
}

// modify this routine to meet your needs
void drawStatus() {
  char tmp[20];

  display.setFont(BASE_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  sprintf_P(tmp, PSTR("WHEEL"));    display.drawStr(4,  10, tmp);
  sprintf_P(tmp, PSTR("MAIN"));     display.drawStr(99, 10, tmp);
  sprintf_P(tmp, PSTR("LEFT"));     display.drawStr(4,  52, tmp);
  sprintf_P(tmp, PSTR("RIGHT"));    display.drawStr(92, 52, tmp);
  sprintf_P(tmp, PSTR("POS"));      display.drawStr(42, 28, tmp);
  sprintf(tmp, "%d", encoderPos);   display.drawStr(68, 28, tmp);

  display.setFont(SMALL_FONT);
  sprintf_P(tmp, stringifyButtonState(wheelBtn));   display.drawStr(4,  23, tmp);
  sprintf_P(tmp, stringifyButtonState(mainBtn));    display.drawStr(99, 23, tmp);
  sprintf_P(tmp, stringifyButtonState(leftBtn));    display.drawStr(4,  40, tmp);
  sprintf_P(tmp, stringifyButtonState(rightBtn));   display.drawStr(99, 40, tmp);
}

// Function used to draw the data on display
void draw() {
  display.firstPage();
  do {
    drawStatus();
  } while(display.nextPage());
}

// Function that plays a tune (melody)
void playTune() {
  for (uint8_t n = 0; n < ArraySize(tune); ++n) {
    uint16_t f = pgm_read_word(&(tune[n].frequency));
    uint8_t d = pgm_read_byte(&(tune[n].duration));
    if (f && d) {
      // if frequency and duration are set, play the tone
      panel.playTone(f, d);
      uint8_t p = pgm_read_byte(&(tune[n].pause));
      delay(p ? p : d);
    }
  }
}

// set everything up
void setup() {
  Serial.begin(57600);
  debugEcho(PSTR("[ Start ]"));
  // I2C scan must show (at least) 2 devices (address 0x3c and 0x3d).
  // If there are no devices reported, check your wiring.
  // If there are more than two devices, this might point to
  // insufficient termination resistors on your SDA/SCL lines.
  scanI2CDevices();
  setupDisplay();   // initialize the display (U8G2 library)
  setupPanel();     // initialize the LeoNerd's Panel
  // enable encoder wheel to beep on rotation only
  panel.setKeyBeep(1400, 10);
  panel.setKeyBeepMask(LeoNerdPanel::WheelRotationBeep);
  draw();
}

void loop() {
  // service the encoder status (must be called from within this loop!)
  panel.loop();
  // read the wheel position and store it
  int8_t v = panel.getWheelCount();
  encoderPos += v;
  // read the status of each button and stor it
  wheelBtn = panel.getButtonState(LeoNerdPanel::WheelButton);
  mainBtn = panel.getButtonState(LeoNerdPanel::MainButton);
  leftBtn = panel.getButtonState(LeoNerdPanel::LeftButton);
  rightBtn = panel.getButtonState(LeoNerdPanel::RightButton);
  // right button plays a tune if held
  if (rightBtn == LongClicked)
      playTune();

  // if the encoder has been turned
  if (encoderPos != lastEncoderPos) {
    lastEncoderPos = encoderPos;
    draw();
    panel.setLED(LeoNerdPanel::Green, encoderPos > 0);    // turn green LED on if wheel position is positive
    panel.setLED(LeoNerdPanel::Red, encoderPos < 0);      // turn red LED on if wheel position is negative
  }
  if (wheelBtn != Open || mainBtn != Open || leftBtn != Open || rightBtn != Open)
    draw();   // draw button states and encoder position on the display
}
