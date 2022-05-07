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
 *  @file   neopixel.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  NeoPixel class header.
 *
 *  This file defines the NeoPixel class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_NEOPIXEL_NEOPIXEL_H_
#define SRC_NEOPIXEL_NEOPIXEL_H_

/*
 *  External headers.
 */
#include <Adafruit_NeoPixel.h>

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Controller;
class Logger;

/**
 *  @brief  NeoPixel class.
 *
 *  This class provides functions for
 *  controlling the NeoPixel LED array.
 */
class NeoPixel
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @param  controller Controller object pointer.
     *  @brief  NeoPixel class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor initializes and turns off
     *  the NeoPixel LED array by default.
     */
    NeoPixel(Logger* logger, Controller* controller);

    /**
     *  @param  brightness LED brightness.
     *  @brief  Set the brightness of the NeoPixel LED array.
     *
     *  This function sets the brightness of the NeoPixel
     *  LED array. The brightness value is constrained between
     *  the minimum and maximum LED brightness values defined
     *  in the PlatformNeoPixel enum.
     */
    void
    setBrightness(const int& brightness);

    /**
     *  @brief  Refresh the NeoPixel LED array.
     *
     *  This function refreshes the NeoPixel LED array based on
     *  the system status. The colors of the LED array reflect
     *  the following system statuses:
     *      - Black: powered off or initializing.
     *      - Blue: standby or safety disengage (large pitch angles).
     *      - Green: active.
     *      - Red: active but with warn or error messages.
     *
     *  This function is expected to be called periodically.
     */
    void
    refresh();

private:

    /**
     *  @param  color Encoded LED RGB color value.
     *  @brief  Set the color of the NeoPixel LED array.
     *
     *  This function sets the color of the NeoPixel LED array.
     *
     *  RGB color value encoding:
     *  https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html
     */
    void
    setColor(const uint32_t& color);

    Logger* logger_;    //!< Logger object pointer.
    Controller* controller_;    //!< Controller object pointer.
    Adafruit_NeoPixel neopixel_;    //!< Adafruit NeoPixel object.
    bool initialized_;  //!< Initialization flag.
    bool last_controller_active_;   //!< Last controller active flag.
};
}   // namespace tumbller

#endif  // SRC_NEOPIXEL_NEOPIXEL_H_