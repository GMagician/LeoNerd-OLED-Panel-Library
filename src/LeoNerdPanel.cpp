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
#include "LeoNerdPanel.h"


LeoNerdPanel::LeoNerdPanel(uint8_t address, int intPin, void (*interruptHandler)(void), void (*eventHandler)(LeoNerdEvent), bool keyBeep) {
  _address = address;

  if (keyBeep)
    setKeyBeep(330, 100);
  _intPin = intPin;
  if (intPin != -1)
    pinMode(intPin, INPUT_PULLUP);
  _interruptHandler = interruptHandler;
  _eventHandler = eventHandler;
  _encoderButton = Button(WheelButton);
  _mainButton = Button(MainButton);
  _leftButton = Button(LeftButton);
  _rightButton = Button(RightButton);
  _maxBrightness = 255;
  _gpioDir = 0;
  _gpioVal = 0;
  _wheelPos = 0;
  _isBusy = false;
}

/**
 * Destructor for clean up
 */
LeoNerdPanel::~LeoNerdPanel() {
  if (_intPin != -1)
    detachInterrupt(_intPin);
#if !defined(ESP32) && !defined(ESP8266)
  if (_I2CBus != nullptr)
    _I2CBus->end();
#endif
}

/**
 * Main routine for reading the events coming from the encoder
 */
void LeoNerdPanel::loop() {
  event_t eventsBuf[eventsQueueSize];

  if (_isBusy)
    return;
  interrupts();
  // if an interrupt pin is assigned and its state is HIGH, there are no events pending
  if (_intPin != -1 && digitalRead(_intPin))
    return;
  _isBusy = true;
  uint8_t eventRead = queryEvents(&eventsBuf[0], eventsQueueSize);
  for (int i = 0; i < eventRead; ++i)
    parseEvent(eventsBuf[i]);
  _isBusy = false;
}

/**
 * Flushes pending events queue
 */
void LeoNerdPanel::flushPendingEvents() {
  // Flush events queue
  if (_intPin != -1) {
    while(!digitalRead(_intPin))
      queryRegister(EventReg);
  }
}

/**
 * Get the state of the button
 *
 * @param button    the button @see button_t
 * @returns the current @see buttonState_t
 */
buttonState_t LeoNerdPanel::getButtonState(button_t button) {
  buttonState_t state;

  switch (button) {
    case WheelButton:
      state = _encoderButton.getState();
      break;
    case MainButton:
      state = _mainButton.getState();
      break;
    case LeftButton:
      state = _leftButton.getState();
      break;
    case RightButton:
      state = _rightButton.getState();
      break;
    default:
      state = Open;
      break;
    }
  if (state != Pressed)
    resetButtonState(button);
  return state;
}

/**
 * Set the KeyBeep frequency and duration
 *
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 *
 */
void LeoNerdPanel::setKeyBeep(int frequency, int duration) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(BeepFreqHReg);
  _I2CBus->write((uint8_t)(frequency >> 8));
  _I2CBus->write((uint8_t)frequency);
  _I2CBus->endTransmission();
  setKeyBeepDuration(duration);
  setKeyBeepMask();
}

/**
 * Set the KeyBeep duration
 * @param duration      the tone duration in milliseconds
 */
void LeoNerdPanel::setKeyBeepDuration(uint8_t duration) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(KeyBeepDurationReg);
  _I2CBus->write((uint8_t)(duration / 10));
  _I2CBus->endTransmission();
}

/**
 * Set the KeyBeep mask
 * @param mask      mask to set (which buttons will beep on action)
 */
void LeoNerdPanel::setKeyBeepMask(beepMask_t mask) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(KeyBeepMaskReg);
  _I2CBus->write(mask);
  _I2CBus->endTransmission();
}

/**
 * Set debounce time
 * @param time      time in milliseconds (0-255)
 */
void LeoNerdPanel::setDebounceTime(uint8_t time) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(DebounceTimeReg);
  _I2CBus->write(time);
  _I2CBus->endTransmission();
}

/**
 * Set the encoder button hold time for "Long Click"
 * @param time  time in centiseconds (0-255)
 */
void LeoNerdPanel::setButtonHoldTime(uint8_t time) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(BtnHoldTimeReg);
  _I2CBus->write(time);
  _I2CBus->endTransmission();
}

/**
 * Set button release mask
 * @param mask  button release mask @see LeoNerdPanel.h
 */
void LeoNerdPanel::setButtonReleaseMask(eventReleaseMask_t mask) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(ReleaseMaskReg);
  _I2CBus->write(mask);
  _I2CBus->endTransmission();
}

/**
 * Switch the given LED on or off
 *
 * @param led       Red or Green
 * @param state     true (on) / false (off)
 */
void LeoNerdPanel::setLED(led_t led, bool state) {
  if (_I2CBus == nullptr)
    return;

  if (led >= 0 && led < MaxLeds) {
    _ledState[led] = state;

    waitIdle();
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(led == Red ? Led1PwmReg : Led2PwmReg);
    _I2CBus->write(state ? _maxBrightness : 0);
    _I2CBus->endTransmission();
  }
}

/**
 * Toggle the given LED on or off
 *
 * @param led       Red or Green
 * @param state     true (on) / false (off)
 */
void LeoNerdPanel::toggleLED(led_t led) {
  if (_I2CBus == nullptr)
    return;

  if (led >= 0 && led < MaxLeds) {
    bool state = _ledState[led] = !_ledState[led];

    waitIdle();
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(led == Red ? Led1PwmReg : Led2PwmReg);
    _I2CBus->write(state ? _maxBrightness : 0);
    _I2CBus->endTransmission();
  }
}

/**
 * Set the given GPIO pin to either INPUT or OUTPUT
 *
 * @param which     number of the GPIO pin (0-3)
 * @param mode      INPUT or OUTPUT
 */
void LeoNerdPanel::setGPIOMode(uint8_t which, int mode) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(GPIOModeReg);
  if (mode == INPUT)
    _gpioDir &= ~(1 << which);
  else
    _gpioDir |= (1 << which);
  _I2CBus->write(_gpioDir & 0x0F);
  _I2CBus->endTransmission();
}

/**
 * Get the given GPIO pin state
 *
 * @param which     number of the GPIO pin (0-3)
 * @returns true for HIGH, false for LOW
 */
bool LeoNerdPanel::getGPIO(uint8_t which) {
  return _gpioVal & (1 << which);
}

/**
 * Set the given GPIO pin state
 *
 * @param which     number of the GPIO pin (0-3)
 * @param state     true (HIGH) / false (LOW)
 */
void LeoNerdPanel::setGPIO(uint8_t which, bool state){
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(GPIOReg);
  if (state)
    _gpioVal |= (1 << which);
  else
    _gpioVal &= ~(1 << which);
  _I2CBus->write(_gpioVal & 0x0F);
  _I2CBus->endTransmission();
}

/**
 * Play a tone
 *
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 *
 */
void LeoNerdPanel::playTone(int16_t frequency, int16_t duration) {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(BeepFreqHReg);
  _I2CBus->write((uint8_t)(frequency >> 8));
  _I2CBus->write((uint8_t)frequency);
  _I2CBus->endTransmission();

  _I2CBus->beginTransmission(_address);
  _I2CBus->write(BeepDurationReg);
  _I2CBus->write((uint8_t)(duration / 10));
  _I2CBus->endTransmission();
}

/**
 * Mute buzzer
 */
void LeoNerdPanel::muteTone() {
  if (_I2CBus == nullptr)
    return;

  waitIdle();
  _I2CBus->beginTransmission(_address);
  _I2CBus->write(BeepDurationReg);
  _I2CBus->write(0);
  _I2CBus->endTransmission();
}

/**
 * Read the RELEASEMASK
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryReleaseMask() {
  waitIdle();
  return queryRegister(ReleaseMaskReg);
}

/**
 * Read the KEYBEEP DURATION
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryKeyBeepDuration() {
  waitIdle();
  return queryRegister(KeyBeepDurationReg);
}

/**
 * Read the KEYBEEP MASK
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryKeyBeepMask() {
  waitIdle();
  return queryRegister(KeyBeepMaskReg);
}

/**
 * Read the BEEP DURATION
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryBeepDuration() {
  waitIdle();
  return queryRegister(BeepDurationReg);
}

/**
 * Read the BEEP TONE
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryBeepTone() {
  waitIdle();
  return queryRegister(BeepToneReg);
}

/**
 * Read the PWM value for LED1
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryLed1Pwm() {
  waitIdle();
  return queryRegister(Led1PwmReg);
}

/**
 * Read the PWM value for LED2
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryLed2Pwm() {
  waitIdle();
  return queryRegister(Led2PwmReg);
}

/**
 * Read the GPIO MODE settings
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryGPIOMode() {
  waitIdle();
  return queryRegister(GPIOModeReg);
}

/**
 * Read the GPIO states
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryGPIO() {
  waitIdle();
  return queryRegister(GPIOReg);
}

/**
 * Read the GPIO PULLUP settings
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryGPIOPullup() {
  waitIdle();
  return queryRegister(GPIOPullUpReg);
}

/**
 * Read the GPIO EVENTMASK settings
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryGPIOEventMask() {
  waitIdle();
  return queryRegister(GPIOEventMaskReg);
}

/**
 * Query the ENCODER ADDRESS
 *
 * @returns the configured address
 */
uint8_t LeoNerdPanel::queryEncoderAddress() {
  waitIdle();
  return queryRegister(EEpromI2CAddressReg);
}

/**
 * Read the OPTIONS register
 *
 * @returns the options as set in EEPROM
 */
uint8_t LeoNerdPanel::queryOptions() {
  waitIdle();
  return queryRegister(EEpromOptionsReg);
}

/**
 * Read the DEBOUNCE TIME
 *
 * @returns the default time
 */
uint8_t LeoNerdPanel::queryDebounceTime() {
  waitIdle();
  return queryRegister(EEpromDebounceReg);
}

/**
 * Read the HOLD TIME
 *
 * @returns the default time
 */
uint8_t LeoNerdPanel::queryHoldTime() {
  waitIdle();
  return queryRegister(EEpromHoldTimeReg);
}

/**
 * Read the BUTTON TO GPIO MAPPING
 *
 * @returns the current GPIO ports it's mapped to
 */
bool LeoNerdPanel::queryButtonMapping(button_t button) {
  waitIdle();
  uint8_t stat = queryRegister(EEpromBtnMappingReg);
  switch (button) {
    case WheelButton:
      return stat & 1;
    case MainButton:
      return stat & 2;
    case LeftButton:
      return stat & 4;
    case RightButton:
      return stat & 8;
    default:
      return false;
  }
}

/**
 * Read the BUTTON TO GPIO MAPPING POLARITY
 *
 * @returns     the current GPIO ports it's mapped to
 */
bool LeoNerdPanel::queryButtonMappingPolarity(button_t button) {
  waitIdle();
  uint8_t stat = queryRegister(EEpromBtnPolarityReg);
  switch (button) {
    case WheelButton:
      return stat & 1;
    case MainButton:
      return stat & 2;
    case LeftButton:
      return stat & 4;
    case RightButton:
      return stat & 8;
    default:
      return false;
  }
}

/**
 * Read the WHEEL ACCELERATION time
 *
 * @returns the default time in ms
 */
uint8_t LeoNerdPanel::queryWheelAcceleration() {
  waitIdle();
  return queryRegister(EEpromAccelerationReg);
}

/**
 * Read the WHEEL DECELERATION time
 *
 * @returns the default time in ms
 */
uint8_t LeoNerdPanel::queryWheelDeceleration() {
  waitIdle();
  return queryRegister(EEPromDecelerationReg);
}

/**
 * Read the VERSION info
 *
 * @returns the current value
 */
uint8_t LeoNerdPanel::queryVersion() {
  waitIdle();
  return queryRegister(SoftwareVersionReg);
}

/**
 * Set EEPROM value
 *
 * Be careful with this method and know what you're doing!
 * It may render your encoder unusable!
 *
 * @param eep_adr   the EEPROM address @see LeoNerdPanel.h (REG_EEPROM*)
 * @param value     the value to be written
 */
void LeoNerdPanel::setEepromValue(i2cRegister_t eep_adr, uint8_t value) {
  if (_I2CBus == nullptr)
    return;

  if (eep_adr >= EEpromI2CAddressReg && eep_adr <= EEPromDecelerationReg) {
    waitIdle();
    unlockEepromWrite();
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(eep_adr);
    _I2CBus->write(value);
    _I2CBus->endTransmission();
  }
}

/**
 * Initialize the encoder library
 */
void LeoNerdPanel::internalBegin(TwoWire *bus) {
  if (bus == nullptr)
    return;

  _I2CBus = bus;
  _I2CBus->begin();
  _wheelPos = 0;
  _isBusy = false;
  flushPendingEvents();
  if (_intPin != -1 && _interruptHandler != nullptr)
    attachInterrupt(_intPin, _interruptHandler, FALLING);
  // set release mask on all buttons by default
  setButtonReleaseMask();
}

/**
 * Parse the event and act sccordingly
 *
 * @param data      the event received
 */
void LeoNerdPanel::parseEvent(event_t event) {
  switch (event.category) {
    case EC_None:
      _wheelPos = 0;
      break;

    case EC_Wheel:
      if (event.wheel.clockWise)
        _wheelPos += (event.wheel.steps + 1);
      else
        _wheelPos -= (event.wheel.steps + 1);
      if (_eventHandler != nullptr)
        _eventHandler(LeoNerdEvent(WheelEvent, event.wheel.clockWise ? ClockwiseTurn : CounterclockwiseTurn));
      break;

    case EC_Button:
      Button *button;
      buttonState_t state;

      switch (event.button.ID) {
        case BI_Wheel:
          button = &_encoderButton;
          break;
        case BI_Main:
          button = &_mainButton;
          break;
        case BI_Left:
          button = &_leftButton;
          break;
        case BI_Right:
          button = &_rightButton;
          break;
        default:
          return;
      }
      switch (event.button.eventID) {
        case BEI_Release:
          state = Released;
          break;
        case BEI_Press:
          state = Pressed;
          break;
        case BEI_Held:
          state = Held;
          break;
        default:
          return;
      }
      setButtonEvent(button, state);
      break;

    case EC_GpIO:
      _gpioVal = event.gpio.status;
      if (_eventHandler != nullptr)
        _eventHandler(LeoNerdEvent(GpioEvent, Open, _gpioVal));
      break;
  }
}

/**
 * Set the button state according to the event received
 *
 * @param button        the button instance
 * @param state         the new button state @see buttonState_t
 */
void LeoNerdPanel::setButtonEvent(Button *button, buttonState_t state) {
  unsigned long now = millis();

  switch (state) {
    case Open:
      button->setState(Open);
    break;

    case Pressed:
      button->setLastPressed(now);
      button->setState(Pressed);
      break;

    case Held:
      button->setState(LongClicked);
      button->resetClickCount();
      if (_eventHandler != nullptr)
        _eventHandler(LeoNerdEvent((eventType_t)button->which(), LongClicked));
      button->setLastReleased(now);
      break;

    case Released:
      if (button->getState() == Pressed) {
        button->resetClickCount();
        button->setState(Clicked);
        button->setLastReleased(now);
        if (_eventHandler != nullptr)
          _eventHandler(LeoNerdEvent((eventType_t)button->which(), Clicked));
      }
      break;

    default:
      break;
  }
}

/**
 * Resets the state of the button
 *
 * @param button    the button to reset @see button_t
 */
void LeoNerdPanel::resetButtonState(button_t button) {
  switch (button) {
    case WheelButton:
      _encoderButton.resetState();
      break;
    case MainButton:
      _mainButton.resetState();
      break;
    case LeftButton:
      _leftButton.resetState();
      break;
    case RightButton:
      _rightButton.resetState();
      break;
    default:
      break;
  }
}

/**
 * Resets the state of all buttons
 */
void LeoNerdPanel::resetButtonsState() {
  _encoderButton.resetState();
  _mainButton.resetState();
  _leftButton.resetState();
  _rightButton.resetState();
}

/**
 * Query the value of the given (FIFO buffered) register
 * @param eventBuf  the pointer to the result buffer
 * @param bufSize   the max. size of the result buffer
 *
 * @returns the number of events received from FIFO; buffer gets filled accordingly
 */
uint8_t LeoNerdPanel::queryEvents(event_t eventsBuf[], uint8_t bufSize) {
  if (_I2CBus == nullptr)
    return 0;

  uint8_t stat = 0;
  do {
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(EventReg);
     stat = _I2CBus->endTransmission();
  } while (stat > 1);
  if (!stat) {
    uint8_t eventsRead = _I2CBus->requestFrom(_address, bufSize);
    while (!_I2CBus->available())
      delayMicroseconds(10);
    uint8_t ndx = 0;
    while (ndx < eventsRead) {
      uint8_t b = _I2CBus->read();
      if (b != 0xFF)
        eventsBuf[ndx++] = event_t(b);
      else
        --eventsRead;
      }
    return eventsRead;
    }
  return 0;
}

/**
 * Query the value of the given register
 * @param reg   the register in charge
 *
 * @returns the response read
 */
uint8_t LeoNerdPanel::queryRegister(i2cRegister_t reg) {
  if (_I2CBus == nullptr)
    return 0;

  uint8_t stat = 0;
  do {
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(reg);
    stat = _I2CBus->endTransmission();
  } while (stat > 1);
  _I2CBus->requestFrom(_address, (uint8_t)1);
  while (!_I2CBus->available())
    delayMicroseconds(10);
  return _I2CBus->read();
}

/**
 * Enable the EEPROM for writing
 *
 * This is a security measure to avoid accidentally overwriting the
 * EEPROM if there're some unwanted signals on the I2C bus.
 * This method is called for each setEepromValue() operation.
 */
void LeoNerdPanel::unlockEepromWrite() {
  if (_I2CBus == nullptr)
    return;

  _I2CBus->beginTransmission(_address);
  _I2CBus->write(EEpromUnlockReg);
  _I2CBus->write(0x12);
  _I2CBus->endTransmission();

  _I2CBus->beginTransmission(_address);
  _I2CBus->write(EEpromUnlockReg);
  _I2CBus->write(0x34);
  _I2CBus->endTransmission();
}
