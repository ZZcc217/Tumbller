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
 *  @file   maneuver_planner.cpp
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Maneuver planner class source.
 *
 *  This file implements the maneuver planner class.
 */

/*
 *  Project headers.
 */
#include "../controller/controller.h"
#include "../controller/controller_data.h"
#include "../utility/logger.h"
#include "../planner/maneuver_planner.h"
#include "../planner/planner_data.h"
#include "../sensor/sensor.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
ManeuverPlanner::ManeuverPlanner(Logger* logger, Sensor* sensor, Controller* controller) : logger_(
        logger), sensor_(sensor), controller_(controller), maneuver_counter_(
        1), maneuver_timer_(0), controller_active_(
        false), plan_started_(false), maneuver_started_(
        false), plan_completed_(false)
{
    /*
     *  Create a set of maneuvers for the example plan.
     *  During their configurations, the maneuvers should
     *  be chained up in a linked list fashion.
     */
    Maneuver* maneuver_1 = new Maneuver();
    Maneuver* maneuver_2 = new Maneuver();
    Maneuver* maneuver_3 = new Maneuver();

    /*
     *  Set the current maneuver to maneuver 1 of the example plan.
     */
    maneuver_ = maneuver_1;

    /*
     *  Example plan maneuver 1 configuration:
     *      - Park for 2 seconds.
     *      - Then, start maneuver 2.
     */
    maneuver_1->type = Maneuver::Type::park;
    maneuver_1->transition_type = Maneuver::TransitionType::duration;
    maneuver_1->transition_value = 2;
    maneuver_1->next = maneuver_2;

    /*
     *  Example plan maneuver 2 configuration:
     *      - Drive forward until the ultrasound distance falls below 0.7 meters.
     *      - Then, start maneuver 3.
     */
    maneuver_2->type = Maneuver::Type::drive;
    maneuver_2->transition_type = Maneuver::TransitionType::ultrasound_below;
    maneuver_2->transition_value = 0.7;
    maneuver_2->next = maneuver_3;

    /*
     *  Example plan maneuver 3 configuration:
     *      - Park for 2 seconds.
     *      - Then, plan completed.
     */
    maneuver_3->type = Maneuver::Type::park;
    maneuver_3->transition_type = Maneuver::TransitionType::duration;
    maneuver_3->transition_value = 2;
    maneuver_3->next = nullptr;

    /*
     *  Using the example plan above, create your own maneuver-based plan.
     *  Feel free to add or remove maneuvers. Also feel free to remove or
     *  comment out the example plan. Just remember to set the current
     *  maneuver to the first maneuver in your own maneuver-based plan.
     *  See the maneuver type and the maneuver transition type enum classes
     *  for all the available maneuver types and maneuver transition types.
     */
    // TODO PART 3 YOUR CODE HERE.
    maneuver_ = maneuver_1;
    maneuver_1->type = Maneuver::Type::park;
    maneuver_1->transition_type = Maneuver::TransitionType::duration;
    maneuver_1->transition_value = 2;
    maneuver_1->next = maneuver_2;

    maneuver_2->type = Maneuver::Type::drive;
    maneuver_2->transition_type = Maneuver::TransitionType::ultrasound_below;
    maneuver_2->transition_value = 0.7;
    maneuver_2->next = maneuver_3;

    maneuver_3->type = Maneuver::Type::reverse;
    maneuver_3->transition_type = Maneuver::TransitionType::ultrasound_above;
    maneuver_3->transition_value = 0.7;
    maneuver_3->next = maneuver_2;
}

void
ManeuverPlanner::plan()
{
    /*
     *  Return if plan has been completed.
     */
    if (plan_completed_)
    {
        return;
    }

    /*
     *  Validate sensor object pointer.
     */
    if (!sensor_)
    {
        logger_->logError("Sensor missing");
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
     *  has not completed, but the current maneuver is null.
     */
    if (!plan_completed_ && plan_started_ && !maneuver_)
    {
        logger_->logInfo("Completed maneuver-based plan");
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
         *  but the current maneuver is already null.
         */
        if (!maneuver_)
        {
            logger_->logError("Empty maneuver-based plan");
            return;
        }

        /*
         *  Mark the plan as started if it has not started.
         */
        logger_->logInfo("Started maneuver-based plan");
        plan_started_ = true;
    }

    if (!maneuver_started_)
    {
        /*
         *  Execute the current maneuver if it has not started.
         *  Generate controller target from the current maneuver,
         *  Set the generated controller target, mark the current
         *  time, and mark the current maneuver as started.
         */
        logger_->logInfo("Started maneuver " + String(maneuver_counter_));
        controller_->setControllerTarget(generateControllerTarget());
        maneuver_timer_ = millis();
        maneuver_started_ = true;
    }
    else
    {
        /*
         *  Transition to the next maneuver based on the
         *  current maneuver transition type and value.
         */
        switch (maneuver_->transition_type)
        {
        case Maneuver::TransitionType::duration:
        {
            /*
             *  If the elapsed duration goes above the transition value from
             *  the current maneuver, transtion to the next maneuver, increment
             *  the maneuver counter, and mark the current maneuver as not started.
             */
            if (static_cast<float>(millis() - maneuver_timer_) >=
                maneuver_->getDurationMillisecond())
            {
                maneuver_ = maneuver_->next;
                maneuver_counter_ ++;
                maneuver_started_ = false;
            }

            break;
        }
        case Maneuver::TransitionType::position_x_below:
        {
            /*
             *  If the current X position goes below the transition value from
             *  the current maneuver, transtion to the next maneuver, increment
             *  the maneuver counter, and mark the current maneuver as not started.
             */
            if (sensor_->getSensorData().position_x < maneuver_->transition_value)
            {
                maneuver_ = maneuver_->next;
                maneuver_counter_ ++;
                maneuver_started_ = false;
            }

            break;
        }
        case Maneuver::TransitionType::position_x_above:
        {
            /*
             *  If the current X position goes above the transition value from
             *  the current maneuver, transtion to the next maneuver, increment
             *  the maneuver counter, and mark the current maneuver as not started.
             */
            if (sensor_->getSensorData().position_x > maneuver_->transition_value)
            {
                maneuver_ = maneuver_->next;
                maneuver_counter_ ++;
                maneuver_started_ = false;
            }

            break;
        }
        case Maneuver::TransitionType::ultrasound_below:
        {
            /*
             *  If the ultrasound distance goes below the transition value from
             *  the current maneuver, transtion to the next maneuver, increment
             *  the maneuver counter, and mark the current maneuver as not started.
             */
            if (sensor_->getSensorData().distance_ultrasound < maneuver_->transition_value)
            {
                maneuver_ = maneuver_->next;
                maneuver_counter_ ++;
                maneuver_started_ = false;
            }

            break;
        }
        case Maneuver::TransitionType::ultrasound_above:
        {
            /*
             *  If the ultrasound distance goes above the transition value from
             *  the current maneuver, transtion to the next maneuver, increment
             *  the maneuver counter, and mark the current maneuver as not started.
             */
            if (sensor_->getSensorData().distance_ultrasound > maneuver_->transition_value)
            {
                maneuver_ = maneuver_->next;
                maneuver_counter_ ++;
                maneuver_started_ = false;
            }

            break;
        }
        default:
        {
            /*
             *  Unknown maneuver transition type. Log warning message.
             */
            logger_->logWarn("Unknown maneuver transition type");
            break;
        }
        }
    }
}

ControllerTarget
ManeuverPlanner::generateControllerTarget() const
{
    /*
     *  Create a default controller target.
     */
    ControllerTarget controller_target;

    /*
     *  Validate current maneuver pointer.
     */
    if (!maneuver_)
    {
        logger_->logWarn("Invalid maneuver");
        return controller_target;
    }

    /*
     *  Generate the controller target based on the
     *  current maneuver type.
     */
    switch (maneuver_->type)
    {
    case Maneuver::Type::park:
    {
        /*
         *  Set the X position controller target to be the current X position,
         *  i.e., stay at the current X position.
         */
        controller_target.position_x = sensor_->getSensorData().position_x; 
        break;
    }
    case Maneuver::Type::reverse:
    {
        /*
         *  Set the X position controller target to be a large negative number,
         *  i.e., reverse until the next maneuver.
         */
        controller_target.position_x = -1000;
        break;
    }
    case Maneuver::Type::reverse_left:
    {
        /*
         *  Set the X position controller target to be 1 meter less than the
         *  current X position, i.e., reverse for 1 meter, and also turn
         *  left 90 degrees.
         */
        controller_target.position_x = sensor_->getSensorData().position_x - 1;
        controller_target.attitude_z = degreesToRadians(90);
        break;
    }
    case Maneuver::Type::reverse_right:
    {
        /*
         *  Set the X position controller target to be 1 meter less than the
         *  current X position, i.e., reverse for 1 meter, and also turn
         *  right 90 degrees.
         */
        controller_target.position_x = sensor_->getSensorData().position_x - 1;
        controller_target.attitude_z = degreesToRadians(-90);
        break;
    }
    case Maneuver::Type::drive:
    {
        /*
         *  Set the X position controller target to be a large positive number,
         *  i.e., drive forward until the next maneuver.
         */
        controller_target.position_x = 1000;
        break;
    }
    case Maneuver::Type::drive_left:
    {
        /*
         *  Set the X position controller target to be 1 meter more than the
         *  current X position, i.e., drive forward for 1 meter, and also turn
         *  left 90 degrees.
         */
        controller_target.position_x = sensor_->getSensorData().position_x + 1;
        controller_target.attitude_z = degreesToRadians(-90);
        break;
    }
    case Maneuver::Type::drive_right:
    {
        /*
         *  Set the X position controller target to be 1 meter more than the
         *  current X position, i.e., drive forward for 1 meter, and also turn
         *  right 90 degrees.
         */
        controller_target.position_x = sensor_->getSensorData().position_x + 1;
        controller_target.attitude_z = degreesToRadians(90);
        break;
    }
    default:
    {
        /*
         *  Unknown maneuver type. Log warning message.
         */
        logger_->logWarn("Unknown maneuver type");
        break;
    }
    }

    /*
     *  Return the generated controller target.
     */
    return controller_target;
}
}   // namespace tumbller