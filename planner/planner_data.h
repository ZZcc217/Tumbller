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
 *  @file   planner_data.h
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Planner data header.
 *
 *  This file defines planner data classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_PLANNER_PLANNER_DATA_H_
#define SRC_PLANNER_PLANNER_DATA_H_

/*
 *  Project headers.
 */
#include "../controller/controller_data.h"
#include "../utility/math.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Planner waypoint struct
 *
 *  This struct contains planner waypoint entries,
 *  such as controller target, waypoint duration,
 *  and the pointer to the next waypoint. The
 *  waypoints can be chained up in a linked list fashion.
 */
struct Waypoint
{
    ControllerTarget controller_target; //!< Controller target.
    float duration; //!< Transition to the next waypoint after this amount of time duration, in seconds.
    Waypoint* next; //!< Pointer to the next waypoint.

    /**
     *  @brief  Planner waypoint struct constructor
     *
     *  This constructor initializes all planner waypoint struct entries.
     */
    Waypoint() : controller_target(), duration(
            0), next(nullptr)
    {
    }

    /**
     *  @return Waypoint duration in milliseconds
     *  @brief  Get waypoint duration in milliseconds
     *
     *  This function returns waypoint duration in milliseconds.
     */
    int
    getDurationMillisecond() const
    {
        return static_cast<int>(duration * 1000);
    }
};

/**
 *  @brief  Planner maneuver struct
 *
 *  This struct contains planner maneuver entries,
 *  such as maneuver type, maneuver transition type,
 *  maneuver transition value and the pointer to the
 *  next maneuver. The maneuvers can be chained up in
 *  a linked list fashion. The struct also defines the
 *  planner maneuver type, and maneuver transition type
 *  enum classes.
 */
struct Maneuver
{
    /**
     *  @brief  Planner maneuver type enum class
     *
     *  This enum class defines all possible maneuver types.
     */
    enum class Type : int
    {
        park = 0,   //!< Park or stop until the end of this maneuver.
        reverse,    //!< Reverse until the end of this maneuver.
        reverse_left,   //!< Reverse to the left until the end of this maneuver.
        reverse_right,  //!< Reverse to the right until the end of this maneuver.
        drive,  //!< Drive forward until the end of this maneuver.
        drive_left, //!< Drive to the left until the end of this maneuver.
        drive_right,    //!< Drive to the right until the end of this maneuver.
    };

    /**
     *  @brief  Planner maneuver transition type enum class
     *
     *  This enum class defines all possible maneuver transition types.
     */
    enum class TransitionType : int
    {
        duration = 0,   //!< Transition to the next maneuver after a certain time duration, in seconds.
        position_x_below,   //!< Transition to the next maneuver if the X position is below a certain value, in meters.
        position_x_above,   //!< Transition to the next maneuver if the X position is above a certain value, in meters.
        ultrasound_below,   //!< Transition to the next maneuver if the ultrasound distance is below a certain value, in meters.
        ultrasound_above    //!< Transition to the next maneuver if the ultrasound distance is above a certain value, in meters.
    };

    Type type;  //!< Maneuver type
    TransitionType transition_type; //!< Maneuver transition type.
    float transition_value; //!< Maneuver transition value, the meaning of which depends on the maneuver transition type.
    Maneuver* next; //!< Pointer to the next maneuver.

    /**
     *  @brief  Planner maneuver struct constructor
     *
     *  This constructor initializes all planner maneuver struct entries.
     */
    Maneuver() : type(Type::park), transition_type(
            TransitionType::duration), transition_value(
            0), next(nullptr)
    {
    }

    /**
     *  @return Maneuver duration in milliseconds
     *  @brief  Get maneuver duration in milliseconds
     *
     *  This function returns maneuver duration in milliseconds.
     *  The return value of this function is meaningful only if
     *  the maneuver transition type is duration.
     */
    int
    getDurationMillisecond() const
    {
        return static_cast<int>(transition_value * 1000);
    }
};
}   // namespace tumbller

#endif  // SRC_PLANNER_PLANNER_DATA_H_