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
 *  @file   timer.h
 *  @author Simon Yu
 *  @date   03/10/2022
 *  @brief  Timer class header.
 *
 *  This file defines the timer class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_UTILITY_TIMER_H_
#define SRC_UTILITY_TIMER_H_

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
 *  @brief  Timer class.
 *
 *  This class provides functions for
 *  generating hardware timer interrupts.
 *
 *  ATmega328P Datasheet:
 *  https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 */
class Timer
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @brief  Timer class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor initializes and configures
     *  the hardware timer 2.
     */
    Timer(Logger* logger);

    /**
     *  @param  period Timer interrupt period, in seconds.
     *  @brief  Set the period of the hardware timer interrupts.
     *
     *  This function sets the period of the hardware timer interrupts.
     */
    void
    setPeriod(const float& period);

    /**
     *  @brief  Enable the hardware timer interrupts.
     *
     *  This function enables the hardware timer interrupts.
     */
    void
    enable();

private:

    Logger* logger_;    //!< Logger object pointer.
};
}   // namespace tumbller

#endif  // SRC_UTILITY_TIMER_H_