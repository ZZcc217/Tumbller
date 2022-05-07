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
 *  @file   math.cpp
 *  @author Simon Yu
 *  @date   01/18/2022
 *  @brief  Math source.
 *
 *  This file implements the Math functions.
 */

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  Project headers.
 */
#include "../utility/math.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
float
degreesToRadians(const float& degrees)
{
    /*
     *  Convert degrees to radians.
     */
    return degrees / 180 * PI;
}

float
radiansToDegrees(const float& radians)
{
    /*
     *  Convert radians to degrees.
     */
    return radians / PI * 180;
}

float
microsecondsToMeters(const float& microseconds, const bool& sound)
{
    const float speed_sound = 343;  // Speed of sound, in meters per second.
    const float speed_light = 299792458;    // Speed of light, in meters per second.

    if (sound)
    {
        /*
         *  Convert microseconds to meters using the speed of sound.
         */
        return speed_sound * microseconds / 1000000;
    }
    else
    {
        /*
         *  Convert microseconds to meters using the speed of light.
         */
        return speed_light * microseconds / 1000000;
    }
}
}   // namespace tumbller