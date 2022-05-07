/*
 *  Copyright (C) 2022  University of Illinois Board of Trustees
 *
 *  Developed by:   Simon Yu (jundayu2@illinois.edu)
 *                  Department of Electrical and Computer Engineering
 *                  https://urldefense.com/v3/__https://www.simonyu.net/__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPEcyYt2p$ 
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
 *  along with this program.  If not, see <https://urldefense.com/v3/__https://www.gnu.org/licenses/__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPLD9wF_H$ >.
 */

/**
 *  @file   actuator.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Actuator class source.
 *
 *  This file implements the actuator class.
 */

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  Project headers.
 */
#include "../actuator/actuator.h"
#include "../utility/logger.h"
#include "../common/motor.h"
#include "../common/pin.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
Actuator::Actuator(Logger* logger) : logger_(logger)
{
    /*
     *  Set pin mode for the motor pins.
     *  See the Pin enum for details.
     */
    // TODO PART 1 YOUR CODE HERE.
    pinMode(Pin::motor_left_pwm, OUTPUT);
    pinMode(Pin::motor_right_pwm, OUTPUT);

    pinMode(Pin::motor_left_direction, OUTPUT);
    pinMode(Pin::motor_right_direction, OUTPUT);

    pinMode(Pin::motor_enable, OUTPUT);
}

ActuationCommand
Actuator::getActuationCommand() const
{
    /*
     *  Return the member actuation command struct.
     */
    // TODO PART 1 YOUR CODE HERE.
    return actuation_command_;
}

void
Actuator::actuate(const ActuationCommand& actuation_command)
{
    /*
     *  Store the given actuation command struct in the
     *  function parameter to the member actuation
     *  command struct.
     */
    // TODO PART 1 YOUR CODE HERE.
    actuation_command_ = actuation_command;

    /*
     *  Write motor enable from the member actuation
     *  command struct to the motor enable pin.
     */
    // TODO PART 1 YOUR CODE HERE.
    digitalWrite(Pin::motor_enable, actuation_command_.motor_enable);

    /*
     *  Write motor directions from the member actuation
     *  command struct to the motor direction pins.
     *
     *  Note that the robot is expected to move forward
     *  with both motor_left_forward and motor_right_forward to be true.
     */
    // TODO PART 1 YOUR CODE HERE.
    digitalWrite(Pin::motor_left_direction, actuation_command_.motor_left_forward);
    digitalWrite(Pin::motor_right_direction, actuation_command_.motor_right_forward);

    /*
     *  Constrain, using the Arduino constrain function, the
     *  motor PWM values from the member actuation command
     *  struct to be in between the minimum and the maximum
     *  PWM values defined in the Motor enum. Then, write
     *  the constrained values to the motor PWM pins.
     *
     *  Note that the PWM values are analog values.
     *  Thus, one should use the Arduino analogWrite
     *  function instead of digitalWrite.
     */
    // TODO PART 1 YOUR CODE HERE.
    actuation_command_.motor_left_pwm = constrain(actuation_command_.motor_left_pwm, Motor::pwm_min, Motor::pwm_max);
    actuation_command_.motor_right_pwm = constrain(actuation_command_.motor_right_pwm, Motor::pwm_min, Motor::pwm_max);
    analogWrite(Pin::motor_left_pwm, actuation_command_.motor_left_pwm);
    analogWrite(Pin::motor_right_pwm, actuation_command_.motor_right_pwm);
}
}   // namespace tumbller