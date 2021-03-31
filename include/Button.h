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
 *
 */
#pragma once

#include <Arduino.h>
#include "ButtonState.h"

class Button {
  public:
    Button() { _which = 0; _state = Open; }
    Button(uint8_t which) { _which = which; _state = Open; _lastPressed = 0L; _lastReleased = 0L; _clickCount = 0; }

    void resetState() { _state = Open; }
    buttonState_t getState() { return _state; }
    void setState(buttonState_t state) { _state = state; }
    unsigned long getLastPressed() { return _lastPressed; }
    void setLastPressed(unsigned long ticks) { _lastPressed = ticks; }
    unsigned long getLastReleased() { return _lastReleased; }
    void setLastReleased(unsigned long ticks) { _lastReleased = ticks; }
    void resetClickCount() { _clickCount = 0; }
    uint8_t which() { return _which; }

  private:
    volatile uint8_t         _which;
    volatile buttonState_t _state;
    unsigned long _lastPressed;
    unsigned long _lastReleased;
    volatile uint8_t _clickCount;
};
