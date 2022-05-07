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
 *  @file   waypoint_planner.cpp
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Waypoint planner class source.
 *
 *  This file implements the waypoint planner class.
 */

/*
 *  Project headers.
 */
#include "../controller/controller.h"
#include "../utility/logger.h"
#include "../utility/math.h"
#include "../planner/planner_data.h"
#include "../sensor/sensor.h"
#include "../planner/waypoint_planner.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
WaypointPlanner::WaypointPlanner(Logger* logger, Sensor* sensor, Controller* controller) : logger_(
        logger), sensor_(sensor), controller_(controller), waypoint_counter_(
        1), waypoint_timer_(0), controller_active_(
        false), plan_started_(false), waypoint_started_(
        false), plan_completed_(false)
{
    /*
     *  Create a set of waypoints for the example plan.
     *  During their configurations, the waypoints should
     *  be chained up in a linked list fashion.
     */
    Waypoint* waypoint_1 = new Waypoint();
    Waypoint* waypoint_2 = new Waypoint();
    Waypoint* waypoint_3 = new Waypoint();
    Waypoint* waypoint_4 = new Waypoint();
    Waypoint* waypoint_5 = new Waypoint();

    /*
     *  Set the current waypoint to waypoint 1 of the example plan.
     */
    waypoint_ = waypoint_1;

    /*
     *  Example plan waypoint 1 configuration:
     *      - Go to 0 meter X position (0 meter forward) without turning for 2 seconds.
     *      - Then, start waypoint 2.
     */
    waypoint_1->controller_target.position_x = 0;
    waypoint_1->controller_target.attitude_z = degreesToRadians(0);
    waypoint_1->duration = 2;
    waypoint_1->next = waypoint_2;

    /*
     *  Example plan waypoint 2 configuration:
     *      - Go to 1 meter X position (1 meter forward) without turning for 10 seconds.
     *      - Then, start waypoint 3.
     */
    waypoint_2->controller_target.position_x = 1;
    waypoint_2->controller_target.attitude_z = degreesToRadians(0);
    waypoint_2->duration = 10;
    waypoint_2->next = waypoint_3;

    /*
     *  Example plan waypoint 3 configuration:
     *      - Go to 2 meter X position (1 meter forward) while turning right 90 degrees for 10 seconds.
     *      - Then, start waypoint 4.
     */
    waypoint_3->controller_target.position_x = 2;
    waypoint_3->controller_target.attitude_z = degreesToRadians(90);
    waypoint_3->duration = 10;
    waypoint_3->next = waypoint_4;

    /*
     *  Example plan waypoint 4 configuration:
     *      - Go to 1 meter X position (1 meter backward) while turning right 90 degrees for 10 seconds.
     *      - Then, start waypoint 5.
     */
    waypoint_4->controller_target.position_x = 1;
    waypoint_4->controller_target.attitude_z = degreesToRadians(-90);
    waypoint_4->duration = 10;
    waypoint_4->next = waypoint_5;

    /*
     *  Example plan waypoint 5 configuration:
     *      - Go to 0 meter X position (1 meter backward) without turning for 10 seconds.
     *      - Then, plan completed.
     */
    waypoint_5->controller_target.position_x = 0;
    waypoint_5->controller_target.attitude_z = degreesToRadians(0);
    waypoint_5->duration = 10;
    waypoint_5->next = nullptr;

    /*
     *  Using the example plan above, create your own waypoint-based plan.
     *  Feel free to add or remove waypoints. Also feel free to remove or
     *  comment out the example plan. Just remember to set the current
     *  waypoint to the first waypoint in your own waypoint-based plan.
     */
    // TODO PART 3 YOUR CODE HERE.
}

void
WaypointPlanner::plan()
{
    /*
     *  Return if plan has been completed.
     */
    if (plan_completed_)
    {
        return;
    }

    /*
     *  Validate controller object pointer.
     */
    if (!controller_)
    {
        logger_->logError("Controller missing");
        return;
    }

    /*
     *  Mark the plan as completed if the plan has started,
     *  has not completed, but the current waypoint is null.
     */
    if (!plan_completed_ && plan_started_ && !waypoint_)
    {
        logger_->logInfo("Completed waypoint-based plan");
        plan_started_ = false;
        plan_completed_ = true;
        return;
    }

    /*
     *  Return if the controller is not active
     *  (pause the plan during safety disengage.)
     */
    if (!controller_active_)
    {
        controller_active_ = controller_->getActiveStatus();
        return;
    }

    if (!plan_started_)
    {
        /*
         *  The plan is empty if the plan has not started
         *  but the current waypoint is already null.
         */
        if (!waypoint_)
        {
            logger_->logError("Empty waypoint-based plan");
            return;
        }

        /*
         *  Mark the plan as started if it has not started.
         */
        logger_->logInfo("Started waypoint-based plan");
        plan_started_ = true;
    }

    if (!waypoint_started_)
    {
        /*
         *  Execute the current waypoint if it has not started.
         *  Set the controller target from the current waypoint,
         *  mark the current time, and mark the current waypoint
         *  as started.
         */
        logger_->logInfo("Started waypoint " + String(waypoint_counter_));
        controller_->setControllerTarget(waypoint_->controller_target);
        waypoint_timer_ = millis();
        waypoint_started_ = true;
    }
    else
    {
        /*
         *  If the elapsed duration goes above the duration from the current
         *  waypoint, transtion to the next waypoint, increment the waypoint
         *  counter, and mark the current waypoint as not started.
         */
        if (static_cast<float>(millis() - waypoint_timer_) >=
                waypoint_->getDurationMillisecond())
        {
            waypoint_ = waypoint_->next;
            waypoint_counter_ ++;
            waypoint_started_ = false;
        }
    }
}
}   // namespace tumbller