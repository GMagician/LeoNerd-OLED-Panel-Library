/**
 * Library for LeoNerd's OLED Panel
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
#pragma once

#include <stdint.h>
#if defined (AVR)
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <avr/pgmspace.h>
#endif
#include "Arduino.h"
#include <Wire.h>
#include "Button.h"
#include "LeoNerdEvent.h"

class LeoNerdPanel {
  public:
    typedef enum : uint8_t {
      WheelButton,
      MainButton,
      LeftButton,
      RightButton
      } button_t;

    typedef enum : uint8_t {
      Red,
      Green,
      MaxLeds
      } led_t;

    typedef enum : uint8_t {
      WheelReleased = 0x02,
      MainReleased = 0x04,
      LeftReleased = 0x08,
      RightReleased = 0x10,
      AllReleased = 0x1E
      } eventReleaseMask_t;

    typedef enum : uint8_t {
      BeepNone = 0x00,
      WheelRotationBeep = 0x01,
      WheelBeep = 0x02,
      MainBeep = 0x04,
      LeftBeep = 0x08,
      RightBeep = 0x10,
      BeepAll = 0x1F
      } beepMask_t;

    // Options Mask
    typedef enum : uint8_t {
      AccelerationOption = 0x01,    // 1=enable / 0=disable encoder wheel acceleration (GMagician only)
      InvertWheelOption= 0x80       // 1=invert encoder wheel direction / 0=default direction
    } options_t;

    // I2C registers
    typedef enum : uint8_t {
      EventReg = 0x01,              // Implements an 8-level deep FIFO of input events. Each event is formed of a single byte.
      ReleaseMaskReg,               // Controls whether buttons will send event reports on release as well as press (bits set to 1), or on press only (bits set to 0).
      DebounceTimeReg,              // Sets the duration in milliseconds for key debounce detection
      BtnHoldTimeReg,               // Sets the duration in centiseconds for reporting a "button held" event (75 by default)
      KeyBeepDurationReg = 0x10,    // Controls the duration of the autonomous keybeep in centiseconds
      KeyBeepMaskReg,               // Controls which button events cause an autonomous keybeep
      BeepDurationReg,              // Sets the duration of the beep tone in centiseconds
      BeepToneReg,                  // Sets the frequency of the beep tone, in units of 10 Hz
      BeepFreqHReg,                 // Sets the frequency of the beep tone, in units of 1 Hz (upper 8 bit)
      BeepFreqLReg,                 // Sets the frequency of the beep tone, in units of 1 Hz (lower 8 bit)
      Led1PwmReg = 0x20,            // Sets the brightness of LED1, which is connected to the red LED in the main button
      Led2PwmReg,                   // Sets the brightness of LED2, which is connected to the green LED in the main button
      GPIOModeReg = 0x30,           // Sets the drive direction on each of the GPIO lines. 0 = input; 1 = output
      GPIOReg,                      // Sets the output state for any GPIO line currently set as output
      GPIOPullUpReg,                // Enables pullup resistors on any of the GPIO lines with bits set high (1)
      GPIOEventMaskReg,             // Sets a mask used for level change event detection on the GPIO lines
      EEpromUnlockReg = 0xBF,       // Virtual register to enable write operations on EEPROM (new for latest version)
      EEpromI2CAddressReg,          // EEPROM storage for encoder I2C address (default: 0x3d)
      EEpromOptionsReg,             // EEPROM storage for misc. options (default: 0x00)
      EEpromDebounceReg,            // EEPROM storage for debounce time (default: 0x14)
      EEpromHoldTimeReg,            // EEPROM storage for hold time (default: 0x4b)
      EEpromBtnMappingReg,          // EEPROM storage for mapping button to GPIO (default: 0x00; 0x04 for SMuFF)
      EEpromBtnPolarityReg,         // EEPROM storage for mapping button to GPIO polarity (default: 0x00; 0x04 for SMuFF)
      EEpromAccelerationReg,        // EEPROM storage for accelaration on encoder wheel (default: 0x25) (GMagican FW only)
      EEPromDecelerationReg,        // EEPROM storage for deceleration on encoder wheel (default: 0x02) (GMagican FW only)
      SoftwareVersionReg = 0xF0     // Reports the current firmware version (default: 0x02)
    } i2cRegister_t;

    LeoNerdPanel(uint8_t address, int intPin = -1, void (*interruptHandler)(void) = nullptr, void (*eventHandler)(LeoNerdEvent) = nullptr, bool keyBeep = false);
    ~LeoNerdPanel();

    void begin(void) { internalBegin(&Wire); }
    void begin(TwoWire *i2cBus) { internalBegin(i2cBus); }
    void loop(void);
    void flushPendingEvents(void);
    int16_t getWheelCount(void) { int16_t val = _wheelPos; _wheelPos = 0; return val; };

    bool busy(void) { return _isBusy; }

    buttonState_t getButtonState(button_t button = WheelButton);
    void setKeyBeep(int frequency, int duration);
    void setKeyBeepDuration(uint8_t duration);
    void setKeyBeepMask(beepMask_t mask = BeepAll);
    void setDebounceTime(uint8_t time = 20);
    void setButtonHoldTime(uint8_t time = 75);
    void setButtonReleaseMask(eventReleaseMask_t mask = AllReleased);

    void playTone(int16_t frequency, int16_t duration);
    void muteTone(void);

    void setMaxBrightness(uint8_t brightness) { _maxBrightness = brightness; }
    void setLED(led_t led, bool state);
    void toggleLED(led_t led);

    void setGPIOMode(uint8_t which, int mode);
    bool getGPIO(uint8_t which);
    void setGPIO(uint8_t which, bool state);

    uint8_t queryReleaseMask(void);
    uint8_t queryKeyBeepDuration(void);
    uint8_t queryKeyBeepMask(void);
    uint8_t queryBeepDuration(void);
    uint8_t queryBeepTone(void);
    uint8_t queryLed1Pwm(void);
    uint8_t queryLed2Pwm(void);
    uint8_t queryGPIOMode(void);
    uint8_t queryGPIO(void);
    uint8_t queryGPIOPullup(void);
    uint8_t queryGPIOEventMask(void);
    uint8_t queryEncoderAddress(void);
    uint8_t queryOptions(void);
    uint8_t queryDebounceTime(void);
    uint8_t queryHoldTime(void);
    bool queryButtonMapping(button_t button);
    bool queryButtonMappingPolarity(button_t button);
    uint8_t queryWheelAcceleration();
    uint8_t queryWheelDeceleration();
    uint8_t queryVersion(void);

    void setEepromValue(i2cRegister_t address, uint8_t value);

  private:
    static constexpr uint8_t eventsQueueSize = 8;   // buffer size of the FIFO

    /**
      * Board event
      */
    typedef enum { EC_None, EC_Wheel, EC_Button, EC_GpIO } eventCategory_t;

    typedef struct {
      uint8_t antiClockWise:1;
      uint8_t clockWise:1;
      uint8_t steps:3;    // 0 or cumulative steps When 'Options.wheelAcc' is enabled
      eventCategory_t category:3;
      } wheelEvent_t;

    typedef enum { BEI_Release, BEI_Press, BEI_Held } buttonEventID_t;
    typedef enum { BI_Wheel, BI_Main, BI_Left, BI_Right } buttonID_t;
    typedef struct {
        buttonEventID_t eventID:2;
        buttonID_t ID:2;
        uint8_t :1;       // reserved
        eventCategory_t category:3;
      } buttonEvent_t;

    typedef struct {
      uint8_t status:4;
      uint8_t :1;         // reserved
      eventCategory_t category:3;
    } gpioEvent_t;

    typedef union _event_t {
      wheelEvent_t wheel;
      buttonEvent_t button;
      gpioEvent_t gpio;
      struct {
        uint8_t :5;       // reserved
        eventCategory_t category:3;
      };
      uint8_t rawData;

      _event_t() {}
      _event_t(uint8_t b) { rawData = b; }
    } event_t;

    void internalBegin(TwoWire *i2cBus);
    void parseEvent(event_t event);
    void setButtonEvent(Button *instance, buttonState_t state);
    void resetButtonState(button_t button);
    void resetButtonsState(void);
    void waitIdle(void) { while(_isBusy); };
    uint8_t queryEvents(event_t eventsBuf[], uint8_t bufSize);
    uint8_t queryRegister(i2cRegister_t reg);
    void unlockEepromWrite();
    void (*_eventHandler)(LeoNerdEvent);
    void (*_interruptHandler)(void);

    TwoWire *_I2CBus;
    uint8_t _address;
    Button _encoderButton;
    Button _mainButton;
    Button _leftButton;
    Button _rightButton;
    uint8_t _maxBrightness;
    uint8_t _gpioDir;
    uint8_t _gpioVal;
    bool _ledState[MaxLeds];
    volatile int16_t _wheelPos;
    int8_t _intPin;
    volatile bool _isBusy;
};
