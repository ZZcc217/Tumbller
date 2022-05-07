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
 *  @file   platform.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Platform header.
 *
 *  This file defines platform-specific classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_COMMON_PLATFORM_H_
#define SRC_COMMON_PLATFORM_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Platform NeoPixel LED array parameter enum.
 *
 *  This enum contains parameters for the platform 
 *  NeoPixel LED array, such as the array size,
 *  maximum LED value, minimum and maximum LED
 *  brightness.
 */
enum PlatformNeoPixel : int
{
    size = 4,   //!< LED array size.
    value_max = 255,    //!< Maximum LED value.
    brightness_min = 10,    //!< Minimum LED brightness.
    brightness_max = 30 //!< Maximum LED brightness.
};

/**
 *  @brief  Platform serial parameter enum.
 *
 *  This enum contains parameters for the platform serial,
 *  such as the baud rate.
 */
enum PlatformSerial : long
{
    baud_rate = 115200  //!< Serial baud rate.
};
}   // namespace tumbller

#endif  // SRC_COMMON_PLATFORM_H_
