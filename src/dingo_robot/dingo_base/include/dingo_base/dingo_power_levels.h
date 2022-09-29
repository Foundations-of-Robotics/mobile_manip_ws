/**
 *  \file       dingo_hardware.h
 *  \brief      Class representing Dingo hardware
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#ifndef DINGO_BASE_DINGO_POWER_LEVELS_H
#define DINGO_BASE_DINGO_POWER_LEVELS_H

/**
 * Container for defining voltage & amperage warning levels for the Dingo-D and Dingo-O
 */
class dingo_power
{
public:
  // battery voltage warning levels
  static constexpr float VOLTAGE_ABSENT                =  1.0;

  // Lithium battery warning levels
  // TODO(civerachb) -- All lithium battery-related code is currently untested, as the hardware to test it on
  // doesn't exist yet!  We'll make any necessary changes to lithium battery support at a later date
  static constexpr float BATTERY_LITHIUM_OVER_VOLT     = 16.9;
  static constexpr float BATTERY_LITHIUM_LOW_VOLT      = 12.0;
  static constexpr float BATTERY_LITHIUM_CRITICAL_VOLT = 11.2;
  // TODO(civerachb) -- test this out on an actual Li battery & adjust
  static constexpr float BATTERY_LITHIUM_LOW_PERCENT   =  0.20;    // 0-1
  // TODO(civerachb) -- test this out on an actual Li battery & adjust
  static constexpr float BATTERY_LITHIUM_CRITICAL_PERCENT = 0.10;  // 0-1

  // SLA battery warning levels
  static constexpr float BATTERY_SLA_OVER_VOLT         = 13.6;
  static constexpr float BATTERY_SLA_LOW_VOLT          = 12.0;
  static constexpr float BATTERY_SLA_CRITICAL_VOLT     = 11.8;

  // user-device power supply voltage warning levels
  static constexpr float USER_VOLT_12_LOW              = 11.0;
  static constexpr float USER_VOLT_5_LOW               =  4.0;
  static constexpr float USER_VOLT_12_HIGH             = 12.5;
  static constexpr float USER_VOLT_5_HIGH              =  5.5;

  // amperage warning levels
  static constexpr float CURRENT_DRAW_CRITICAL         = 28.0;
  static constexpr float CURRENT_DRAW_WARNING          = 18.0;
  static constexpr float CURRENT_DRAW_REMINDER         =  8.0;

  // temperature warnings for PCB & MCU
  static constexpr float TEMPERATURE_CRITICAL          = 100.0;
  static constexpr float TEMPERATURE_WARNING           =  60.0;
};
#endif  // DINGO_BASE_DINGO_POWER_LEVELS_H
