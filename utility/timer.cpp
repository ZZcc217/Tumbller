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
 *  @file   timer.cpp
 *  @author Simon Yu
 *  @date   03/10/2022
 *  @brief  Timer class source.
 *
 *  This file implements the timer class.
 */

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  Project headers.
 */
#include "../utility/logger.h"
#include "../utility/timer.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
Timer::Timer(Logger* logger) : logger_(logger)
{
    /*
     *  Initialize hardware timer 2 TCCR2A register.
     */
    TCCR2A = 0;

    /*
     *  Initialize hardware timer 2 TCCR2B register.
     */
    TCCR2B = 0;

    /*
     *  Set hardware timer 2 counter to 0.
     */
    TCNT2  = 0;

    /*
     *  Set hardware timer 2 output comparator register to 0.
     */
    OCR2A = 0;

    /*
     *  Set hardware timer 2 to CTC mode.
     */
    TCCR2A |= (1 << WGM21);

    /*
     *  Set hardware timer 2 to 1024 prescaler.
     */
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
}

void
Timer::setPeriod(const float& period)
{
    const float frequency_arduino = 16000000;   // Arduino clock speed, in Hz.
    const float prescaler = 1024; // Hardware timer 2 prescaler.
    const float frequency_period = 1 / period;  // Frequency of the given period, in Hz.

    /*
     *  Set hardware timer 2 output comparator register.
     */
    OCR2A = static_cast<int>(frequency_arduino / prescaler / frequency_period);
}

void
Timer::enable()
{
    /*
     *  Enable hardware timer 2 output comparator interrupt.
     */
    TIMSK2 |= (1 << OCIE2A);
}
}   // namespace tumbller