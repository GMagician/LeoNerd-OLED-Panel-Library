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

#include <Arduino.h>
#include "ButtonState.h"

// Event types
typedef enum {
  None = -1,
  WheelEvent,
  WheelButtonEvent,
  MainButtonEvent,
  LeftButtonEvent,
  RightButtonEvent,
  GpioEvent
} eventType_t;


class LeoNerdEvent {
  public:
    LeoNerdEvent() { EvtType = None; State = Open; Value = 0;}
    LeoNerdEvent(eventType_t event, buttonState_t state, uint8_t value = 0) { EvtType = event; State = state; Value = value; }

    volatile eventType_t EvtType;
    volatile buttonState_t State;
    volatile uint8_t Value;
};
