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
 *  @file   neopixel.cpp
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  NeoPixel class source.
 *
 *  This file implements the NeoPixel class.
 */

/*
 *  Project headers.
 */
#include "../controller/controller.h"
#include "../utility/logger.h"
#include "../neopixel/neopixel.h"
#include "../common/pin.h"
#include "../common/platform.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
NeoPixel::NeoPixel(Logger* logger, Controller* controller) : logger_(
        logger), controller_(controller), neopixel_(
        PlatformNeoPixel::size, Pin::neopixel, 
        NEO_GRB + NEO_KHZ800), initialized_(
        false), last_controller_active_(false)
{
    /*
     *  Initialize NeoPixel LED array.
     */
    neopixel_.begin();

    /*
     *  Set the brightness of the NeoPixel LED array to maximum.
     */
    setBrightness(PlatformNeoPixel::brightness_max);

    /*
     *  Set the color of the NeoPixel LED array to black (turned off).
     */
    setColor(0);
}

void
NeoPixel::setBrightness(const int& brightness)
{
    /*
     *  Constrain the brightness value between the minimum
     *  and maximum LED brightness values defined in the
     *  PlatformNeoPixel enum.
     */
    if (brightness < PlatformNeoPixel::brightness_min)
    {
        neopixel_.setBrightness(PlatformNeoPixel::brightness_min);
    }
    else if (brightness > PlatformNeoPixel::brightness_max)
    {
        neopixel_.setBrightness(PlatformNeoPixel::brightness_max);
    }
    else
    {
        neopixel_.setBrightness(brightness);
    }
}

void
NeoPixel::refresh()
{
    /*
     *  Get controller active status.
     */
    bool controller_active = controller_->getActiveStatus();

    /*
     *  Get logger status.
     */
    Logger::LogLevel logger_status = logger_->getStatus();

    /*
     *  Validate controller object pointer.
     */
    if (!controller_)
    {
        logger_->logError("Controller missing");
    }
    else
    {
        if (!initialized_)
        {
            /*
             *  Refresh the array unconditionally, if uninitialized.
             */
            if (controller_active)
            {
                if (logger_status == Logger::LogLevel::warn || logger_status == Logger::LogLevel::error)
                {
                    /*
                     *  Set the color of the array to red
                     *  if there are any warn or error messages.
                     */
                    setColor(neopixel_.Color(255, 0, 0));
                }
                else
                {
                    /*
                     *  Set the color of the array to green otherwise.
                     */
                    setColor(neopixel_.Color(0, 255, 0));
                }
            }
            else if (!controller_active)
            {
                /*
                 *  Set the color of the array to blue
                 *  if thecontroller is not active (safety disengage).
                 */
                setColor(neopixel_.Color(0, 0, 255));
            }

            initialized_ = true;
        }
        else
        {
            /*
             *  Refresh the array only on change in
             *  controller active status, if initialized.
             */
            if (!last_controller_active_ && controller_active)
            {
                if (logger_status == Logger::LogLevel::warn || logger_status == Logger::LogLevel::error)
                {
                    /*
                     *  Set the color of the array to red
                     *  if there are any warn or error messages.
                     */
                    setColor(neopixel_.Color(255, 0, 0));
                }
                else
                {
                    /*
                     *  Set the color of the array to green otherwise.
                     */
                    setColor(neopixel_.Color(0, 255, 0));
                }
            }
            else if (last_controller_active_ && !controller_active)
            {
                /*
                 *  Set the color of the array to blue
                 *  if thecontroller is not active (safety disengage).
                 */
                setColor(neopixel_.Color(0, 0, 255));
            }
        }
    }

    /*
     *  Update last controller active flag.
     */
    last_controller_active_ = controller_active;
}

void
NeoPixel::setColor(const uint32_t& color)
{
    /*
     *  Set the color value of each LED in the array.
     */
    for (int i = 0; i < PlatformNeoPixel::size; i ++)
    {
        neopixel_.setPixelColor(i, color);
    }

    /*
     *  Apply the set color values.
     */
    neopixel_.show();
}
}   // namespace tumbller