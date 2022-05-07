/*
 *  Copyright (C) 2022  University of Illinois Board of Trustees
 *
 *  Developed by:   Simon Yu (jundayu2@illinois.edu)
 *                  Department of Electrical and Computer Engineering
 *                  https://www.simonyu.net/
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *  @file   pin.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Pin header.
 *
 *  This file defines pin-specific classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_COMMON_PIN_H_
#define SRC_COMMON_PIN_H_

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Pin assignment enum.
 *
 *  This enum contains the pin assignments.
 */
enum Pin : int
{
    motor_left_encoder = 2, //!< (Input) Left motor encoder.
    neopixel = 3,   //!< (Output) NeoPixel LED array.
    motor_right_encoder = 4,    //!< (Input) Right motor encoder.
    motor_left_pwm = 5, //!< (Output) Left motor PWM value.
    motor_right_pwm = 6,    //!< (Output) Right motor PWM value.
    motor_right_direction = 7,  //!< (Output) Right motor direction.
    motor_enable = 8,   //!< (Output) Motor enable.
    infrared_ping = 9,  //!< (Output) Infrared ping.
    pushbutton = 10,    //!< (Input) Pushbutton.
    ultrasound_ping = 11,   //!< (Output) Ultrasound ping.
    motor_left_direction = 12,  //!< (Output) Left motor direction.
    infrared_left_pong = A0,    //!< (Input) Left infrared pong.
    infrared_right_pong = A1,   //!< (Input) Right infrared pong.
    battery_voltage = A2,   //!< (Input) Battery voltage.
    ultrasound_pong = A3,   //!< (Input) Ultrasound pong.
};
}   // namespace tumbller

#endif  // SRC_COMMON_PIN_H_
