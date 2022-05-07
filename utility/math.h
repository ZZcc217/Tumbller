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
 *  @file   math.h
 *  @author Simon Yu
 *  @date   01/18/2022
 *  @brief  Math header.
 *
 *  This file defines the Math functions.
 */

/*
 *  Include guard.
 */
#ifndef SRC_UTILITY_MATH_H_
#define SRC_UTILITY_MATH_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @param  degrees Data in degrees.
 *  @return Data in radians.
 *  @brief  Convert degrees to radians.
 *
 *  This function converts data from degrees to radians.
 */
float
degreesToRadians(const float& degrees);

/**
 *  @param  radians Data in radians.
 *  @return Data in degrees.
 *  @brief  Convert radians to degrees.
 *
 *  This function converts data from radians to degrees.
 */
float
radiansToDegrees(const float& radians);

/**
 *  @param  microseconds Data in microseconds.
 *  @param  sound Whether the data is for a sound wave.
 *  @return Data in meters.
 *  @brief  Convert microseconds to meters.
 *
 *  This function converts data from microseconds to meters
 *  using either the speed of sound or the speed of light.
 */
float
microsecondsToMeters(const float& microseconds, const bool& sound);
}   // namespace tumbller

#endif  // SRC_UTILITY_MATH_H_