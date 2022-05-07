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
 *  @file   waypoint_planner.h
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Waypoint planner class header.
 *
 *  This file defines the waypoint planner class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_PLANNER_WAYPOINT_PLANNER_H_
#define SRC_PLANNER_WAYPOINT_PLANNER_H_

/*
 *  Project headers.
 */
#include "../planner/planner.h"

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
class Sensor;
struct Waypoint;

/**
 *  @brief  Waypoint planner class.
 *
 *  This class provides functions for creating and
 *  executing a waypoint-based plan. The class
 *  inherits the planner abstract class.
 */
class WaypointPlanner : public Planner
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @param  sensor Sensor object pointer.
     *  @param  controller Controller object pointer.
     *  @brief  Waypoint planner class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor defines a waypoint-based plan.
     */
    WaypointPlanner(Logger* logger, Sensor* sensor, Controller* controller);

    /**
     *  @brief  Execute the defined waypoint-based plan.
     *
     *  This function executes a waypoint-based plan
     *  defined by the constructor. The function implements its
     *  pure abstract parent in the planner abstract class.
     *  This function is expected to be called periodically.
     */
    void
    plan() override;

private:

    Logger* logger_;    //!< Logger object pointer.
    Sensor* sensor_;    //!< Sensor object pointer.
    Controller* controller_;    //!< Controller object pointer.
    Waypoint* waypoint_;    //!< Current waypoint pointer.
    int waypoint_counter_;  //!< Waypoint counter.
    unsigned long waypoint_timer_;  //!< Waypoint timer, in seconds.
    volatile bool controller_active_;   //!< Controller active flag.
    volatile bool plan_started_;    //!< Plan started flag.
    volatile bool waypoint_started_;    //!< Waypoint started flag.
    volatile bool plan_completed_;  //!< Plan completed flag.
};
}   // namespace tumbller

#endif  // SRC_PLANNER_WAYPOINT_PLANNER_H_