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
 *  @file   controller.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Controller class header.
 *
 *  This file defines the controller class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_CONTROLLER_CONTROLLER_H_
#define SRC_CONTROLLER_CONTROLLER_H_

/*
 *  Project headers.
 */
#include "../actuator/actuator_data.h"
#include "../controller/controller_data.h"
#include "../controller/pid_controller.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Logger;
struct SensorData;

/**
 *  @brief  Controller class.
 *
 *  This class provides functions for establishing a control system,
 *  which takes sensor data and outputs an actuation command.
 */
class Controller
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @brief  Controller class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor initializes and configures
     *  all of its PID controllers.
     */
    Controller(Logger* logger);

    /**
     *  @return Actuation command struct.
     *  @brief  Get the class member actuation command strut.
     *
     *  This function returns the class member actuation command strut.
     */
    ActuationCommand
    getActuationCommand() const;

    /**
     *  @return Controller active flag.
     *  @brief  Get the controller active status.
     *
     *  This function returns the controller active status. If inactive,
     *  the controller is in a safety disengage and generates an actuation
     *  command that stops the motors.
     */
    bool
    getActiveStatus() const;

    /**
     *  @return Controller target struct.
     *  @brief  Get the class member controller target strut.
     *
     *  This function returns the class member controller target strut.
     */
    ControllerTarget
    getControllerTarget() const;

    /**
     *  @param  controller_target Controller target struct.
     *  @brief  Set the controller target.
     *
     *  This function sets the controller target.
     */
    void
    setControllerTarget(const ControllerTarget& controller_target);

    /**
     *  @param  period Control time period, in seconds.
     *  @param  fast_domain Whether the given control time period is for fast domain.
     *  @brief  Set the control time period.
     *
     *  This function sets the control time period, for fast or slow
     *  domain, for the control system.
     */
    void
    setPeriod(const float& period, const bool& fast_domain);

    /**
     *  @param  sensor_data Sensor data struct.
     *  @param  fast_domain Whether to perform fast domain control.
     *  @brief  Execute the control system.
     *
     *  This function executes the control system, for fast or slow
     *  domain, which takes sensor data and populates the member
     *  actuation command struct. This function is expected to be
     *  called periodically.
     */
    void
    control(const SensorData& sensor_data, const bool& fast_domain);

private:

    Logger* logger_;    //!< Logger object pointer.
    ControllerTarget controller_target_;    //!< Controller target struct.
    ActuationCommand actuation_command_;    //!< Actuation command struct.
    PIDController pid_controller_position_x_;   //!< X position (forward/backward) PID controller object.
    PIDController pid_controller_attitude_y_;   //!< Y attitude (pitch) PID controller object.
    PIDController pid_controller_attitude_z_;   //!< Z attitude (yaw) PID controller object.
    PIDController::Gain pid_controller_gain_position_x_;    //!< X position (forward/backward) PID controller gain struct.
    PIDController::Gain pid_controller_gain_attitude_y_;    //!< Y attitude (pitch) PID controller gain struct.
    PIDController::Gain pid_controller_gain_attitude_z_;    //!< Z attitude (yaw) PID controller gain struct.
    PIDController::Saturation pid_controller_saturation_position_x_;    //!< X position (forward/backward) PID controller saturation struct.
    PIDController::Saturation pid_controller_saturation_attitude_y_;    //!< Y attitude (pitch) PID controller saturation struct.
    PIDController::Saturation pid_controller_saturation_attitude_z_;    //!< Z attitude (yaw) PID controller saturation struct.
    float output_position_x_;   //!< X position (forward/backward) PID controller output.
    float output_attitude_y_;   //!< Y attitude (pitch) PID controller output.
    float output_attitude_z_;   //!< Z attitude (yaw) PID controller output.
    bool active_;   //!< Controller active flag.
};
}   // namespace tumbller

#endif  // SRC_CONTROLLER_CONTROLLER_H_
