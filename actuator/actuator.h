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
 *  @file   actuator.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Actuator class header.
 *
 *  This file defines the actuator class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_ACTUATOR_ACTUATOR_H_
#define SRC_ACTUATOR_ACTUATOR_H_

/*
 *  Project headers.
 */
#include "../actuator/actuator_data.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Logger;

/**
 *  @brief  Actuator class.
 *
 *  This class provides functions for commanding actuators,
 *  such as the motors.
 */
class Actuator
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @brief  Actuator class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor initializes the related Arduino
     *  I/O pins.
     */
    Actuator(Logger* logger);

    /**
     *  @return Actuation command struct.
     *  @brief  Get the class member actuation command strut.
     *
     *  This function returns the class member actuation command strut.
     */
    ActuationCommand
    getActuationCommand() const;

    /**
     *  @param  actuation_command actuation command struct.
     *  @brief  Actuation function.
     *
     *  This function takes an actuation command struct
     *  and performs actuation. This function is expected
     *  to be called periodically.
     */
    void
    actuate(const ActuationCommand& actuation_command);

private:

    Logger* logger_;    //!< Logger object pointer.
    ActuationCommand actuation_command_;    //!< Actuation command struct.
};
}   // namespace tumbller

#endif  // SRC_ACTUATOR_ACTUATOR_H_