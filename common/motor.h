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
 *  @file   motor.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Motor header.
 *
 *  This file defines motor-specific classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_COMMON_MOTOR_H_
#define SRC_COMMON_MOTOR_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Motor parameter enum.
 *
 *  This enum contains motor parameters such as
 *  the minimum and maximum pulse width modulation
 *  (PWM) values.
 */
enum Motor : int
{
    pwm_min = 0,    //!< Minimum PWM value.
    pwm_max = 255   //!< Maximum PWM value.
};
}   // namespace tumbller

#endif  // SRC_COMMON_MOTOR_H_