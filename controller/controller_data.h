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
 *  @file   controller_data.h
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Controller data header.
 *
 *  This file defines controller data classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_CONTROLLER_CONTROLLER_DATA_H_
#define SRC_CONTROLLER_CONTROLLER_DATA_H_

/*
 *  Project headers.
 */
#include "../utility/math.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Controller target struct.
 *
 *  This struct contains controller target entries,
 *  such as X position, Y attitude (pitch), and
 *  Z attitude (yaw) targets.
 */
struct ControllerTarget
{
    float position_x;   //!< X position controller target, in meters.
    float attitude_y;   //!< Y attitude (pitch) controller target, in radians.
    float attitude_z;   //!< Z attitude (yaw) controller target, in radians.

    /**
     *  @brief  Controller target struct constructor.
     *
     *  This constructor initializes all controller target struct entries to 0.
     */
    ControllerTarget() : position_x(0), attitude_y(
            degreesToRadians(0)), attitude_z(degreesToRadians(
            0))
    {
    }
};
}   // namespace tumbller

#endif  // SRC_CONTROLLER_CONTROLLER_DATA_H_