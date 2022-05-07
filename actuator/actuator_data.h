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
 *  @file   actuator_data.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Actuator data header.
 *
 *  This file defines actuator data classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_ACTUATOR_ACTUATOR_DATA_H_
#define SRC_ACTUATOR_ACTUATOR_DATA_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Actuation command struct.
 *
 *  This struct contains actuation command entries,
 *  such as the motor enable, left and right motor directions
 *  and pulse width modulation (PWM) values.
 */
struct ActuationCommand
{
    bool motor_enable;  //!< Motor enable.
    bool motor_left_forward;    //!< Left motor direction.
    bool motor_right_forward;   //!< Right motor direction.
    float motor_left_pwm;   //!< Left motor PWM value.
    float motor_right_pwm;  //!< Right motor PWM value.

    /**
     *  @brief  Actuation command struct constructor.
     *
     *  This constructor initializes all actuation command struct entries.
     */
    ActuationCommand() : motor_enable(false), motor_left_forward(
            true), motor_right_forward(false), motor_left_pwm(
            0), motor_right_pwm(0)
    {
    }
};
}   // namespace tumbller

#endif  // SRC_ACTUATOR_ACTUATOR_DATA_H_