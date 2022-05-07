/*
 *  Copyright (C) 2022  University of Illinois Board of Trustees
 *
 *  Developed by:   Simon Yu (jundayu2@illinois.edu)
 *                  Department of Electrical and Computer Engineering
 *                  https://urldefense.com/v3/__https://www.simonyu.net/__;!!DZ3fjg!s3kgEcPI6nVTJcr3Rk_6dxTTkYb69bmU-b6AHbdKAa2F7t9QbgRaEOsJnGOvCPoVjqiy$ 
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
 *  along with this program.  If not, see <https://urldefense.com/v3/__https://www.gnu.org/licenses/__;!!DZ3fjg!s3kgEcPI6nVTJcr3Rk_6dxTTkYb69bmU-b6AHbdKAa2F7t9QbgRaEOsJnGOvCOZ71LDQ$ >.
 */

/**
 *  @file   controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Controller class source.
 *
 *  This file implements the controller class.
 */

/*
 *  Project headers.
 */
#include "../common/motor.h"
#include "../controller/controller.h"
#include "../utility/logger.h"
#include "../utility/math.h"
#include "../sensor/sensor.h"
#include "../sensor/sensor_data.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
Controller::Controller(Logger* logger) : logger_(logger), controller_target_(
        ), pid_controller_position_x_(logger), pid_controller_attitude_y_(
        logger), pid_controller_attitude_z_(logger), output_position_x_(
        0), output_attitude_y_(0), output_attitude_z_(0), active_(
        false)
{
    /*
     *  Populate X position (forward/backward) PID controller gain struct.
     *
     *  Tune this controller only after successfully tuning the Y attitude
     *  (pitch) controller. Before that, set all gains of this controller to 0.
     *
     *  For this controller, all gains should be positive. If your system
     *  diverges with positive gains, make sure you have all sensor data
     *  in the standard body reference frame, and you have subtracted state
     *  with target (Y - R), not the other way around, in the PID
     *  controller control function.
     *
     *  While tuning, start increasing the differential gain first. After the
     *  controller is able resist some velocity change (damping),
     *  then start increasing the proportional gain until the robot is able to
     *  come back when pushed away (X position control).
     *
     *  The integral gain tuning is optional (setting it and the max
     *  integral to 0 would still work,) and the open loop gain should be 0.
     *
     *  See Lecture 13 slide 9 for the PID controller tuning heuristics.
     */
    // TODO PART 2 YOUR CODE HERE.
    pid_controller_gain_position_x_.proportional = 0;
    pid_controller_gain_position_x_.differential = 0;
    pid_controller_gain_position_x_.integral = 0;
    pid_controller_gain_position_x_.integral_max = 0;
    pid_controller_gain_position_x_.open_loop = 0;


    /*
     *  Populate Y attitude (pitch) PID controller gain struct.
     *
     *  Tune this controller first. Before start, set all the gains of
     *  all controllers to 0, including this one.
     *
     *  For this controller, all gains should be negative (except for
     *  max integral). If your system diverges with positive gains, make
     *  sure you have all sensor data in the standard body reference frame,
     *  and you have subtracted state with target (Y - R), not the other
     *  way around, in the PID controller control function.
     *
     *  While tuning, start with the proportional gain first. Increase the
     *  magnitude of the proportional gain until the robot starts mildly
     *  shaking around the Y axis (marginally stable). Then, start
     *  increasing the magnitude of the differential gain until the shaking
     *  disappears (damping).
     *
     *  The integral gain tuning is optional (setting it and the max
     *  integral to 0 would still work,) and the open loop gain should be 0.
     *
     *  See Lecture 13 slide 9 for the PID controller tuning heuristics.
     */
    // TODO PART 2 YOUR CODE HERE.
    pid_controller_gain_attitude_y_.proportional = -4500;
    pid_controller_gain_attitude_y_.differential = -60;
    pid_controller_gain_attitude_y_.integral = 0;
    pid_controller_gain_attitude_y_.integral_max = 0;
    pid_controller_gain_attitude_y_.open_loop = 0;
    

    /*
     *  Populate Z attitude (yaw) PID controller gain struct.
     *
     *  Tune this controller last. By now, you should have sucessfully
     *  tuned the other two controllers. Also, you should have sucessfully
     *  tuned the input saturation bounds below. Before that, set all gains
     *  of this controller to 0.
     *
     *  For this controller, all gains should be positive. If your system
     *  diverges with positive gains, make sure you have all sensor data
     *  in the standard body reference frame, and you have subtracted
     *  state with target (Y - R), not the other way around, in the PID
     *  controller control function.
     *
     *  While tuning, start with the open loop gain first. Increase the
     *  magnitude of the open loop gain until the robot is able to turn right
     *  to 90 degrees under the command of the waypoint planner example plan
     *  (open loop yaw control). Then, increase the magnitude of the
     *  differential gain until the shakiness, if any, during the turning
     *  disappears (damping).
     *
     *  Both integral gain and the proportional gain should be 0 since this
     *  is an open loop controller.
     *
     *  Why do we use an open loop controller instead of a closed loop PID
     *  controller for the yaw control? The reason is that the MPU does not
     *  have a compass, which means that we do not have a reliable yaw
     *  feedback data to control the yaw in a closed loop. Thus, we had to
     *  resort to using open loop control for the yaw.
     */
    // TODO PART 3 YOUR CODE HERE.
    pid_controller_gain_attitude_z_.proportional = 0;
    pid_controller_gain_attitude_z_.differential = 0;
    pid_controller_gain_attitude_z_.integral = 0;
    pid_controller_gain_attitude_z_.integral_max = 0;
    pid_controller_gain_attitude_z_.open_loop = 0;
    

    /*
     *  Set the PID controller gain structs to
     *  their respective PID controllers.
     */
    // TODO PART 2 YOUR CODE HERE.
    pid_controller_position_x_.setGain(pid_controller_gain_position_x_);
    pid_controller_attitude_y_.setGain(pid_controller_gain_attitude_y_);
    pid_controller_attitude_z_.setGain(pid_controller_gain_attitude_z_);

    /*
     *  Adjust the input saturation bounds such that your robot
     *  is able to go forward and backward with moderate speed
     *  under the command of the waypoint planner example plan.
     *
     *  Tune this saturation bounds first and test it with the
     *  waypoint planner before tuning the Z attitude (yaw)
     *  controller gains.
     *
     *  The lower bound controls the forward saturation and should
     *  be a negative value.The upper bound controls the backward
     *  saturation and should be a positive value. The bounds in
     *  turn also control the velocity of the robot (too fast the
     *  robot would fail to balance, too slow the robot would fail
     *  to overcome friction. Thus it's up to you to find the
     *  suitable value for your robot.)
     *
     *  Why does the controller input need to be saturated? Let's
     *  say your robot is current at 0 meter X position, and your
     *  target is 1 meter. The magnitude of the error (e) would
     *  be 1 meter. However, if your target is instead 1000 meters,
     *  then the magnitude of the error (e) has blown up, thus needs
     *  to be saturated within a limited range to prevent excess
     *  control output.
     */
    // TODO PART 3 YOUR CODE HERE.
    pid_controller_saturation_position_x_.saturation_input_lower = -5;
    pid_controller_saturation_position_x_.saturation_input_upper = 5;
    pid_controller_saturation_attitude_y_.saturation_input_lower = -1;
    pid_controller_saturation_attitude_y_.saturation_input_upper = 1;
    pid_controller_saturation_attitude_z_.saturation_input_lower = 0;
    pid_controller_saturation_attitude_z_.saturation_input_upper = 0;


    /*
     *  Set the PID controller saturation structs to
     *  their respective PID controllers.
     */
    // TODO PART 3 YOUR CODE HERE.
    pid_controller_position_x_.setSaturation(pid_controller_saturation_position_x_);
    pid_controller_attitude_y_.setSaturation(pid_controller_saturation_attitude_y_);
    pid_controller_attitude_z_.setSaturation(pid_controller_saturation_attitude_z_);


    /*
     *  Set the current default controller target.
     */
    // TODO PART 2 YOUR CODE HERE.
    controller_target_ = ControllerTarget();
}

ActuationCommand
Controller::getActuationCommand() const
{
    /*
     *  Return the class member actuation command strut.
     */
    // TODO PART 2 YOUR CODE HERE.
    return actuation_command_;
}

bool
Controller::getActiveStatus() const
{
    /*
     *  Return the controller active flag.
     */
    // TODO PART 2 YOUR CODE HERE.
    return active_;
}

ControllerTarget
Controller::getControllerTarget() const
{
    /*
     *  Return the class member controller target strut.
     */
    // TODO PART 2 YOUR CODE HERE.
    return controller_target_;
}

void
Controller::setControllerTarget(const ControllerTarget& controller_target)
{
    /*
     *  Store the given controller target struct in the
     *  function parameter to the member controller
     *  target struct.
     */
    // TODO PART 2 YOUR CODE HERE.
    controller_target_ = controller_target;

    /*
     *  Set the entries in the member controller target
     *  struct to their respective PID controllers.
     */
    // TODO PART 2 YOUR CODE HERE.
    pid_controller_position_x_.setTarget(controller_target_.position_x);
    pid_controller_attitude_y_.setTarget(controller_target_.attitude_y);
    pid_controller_attitude_z_.setTarget(controller_target_.attitude_z);
}

void
Controller::setPeriod(const float& period, const bool& fast_domain)
{
    /*
     *  The reason behind the fast and slow time domain here is
     *  similar to the explanation in the sensor class.
     *  Controlling Y attitude (pitch) requires little to no
     *  delay and thus requires fast domain. Controlling X position
     *  (forward/backward) and Z attitude (yaw), however, the slow
     *  domain is more suitable as their sensor data is noisier.
     */
    if (fast_domain)
    {
        /*
         *  Set the Y attitude (pitch) PID controller period.
         */
         // TODO PART 2 YOUR CODE HERE.
         pid_controller_attitude_y_.setPeriod(period);
    }
    else
    {
        /*
         *  Set the X position (forward/backward) PID controller period.
         */
         // TODO PART 2 YOUR CODE HERE.
         pid_controller_position_x_.setPeriod(period);

        /*
         *  Set the Z attitude (yaw) PID controller period.
         */
         // TODO PART 2 YOUR CODE HERE.
         pid_controller_attitude_z_.setPeriod(period);
    }
}

void
Controller::control(const SensorData& sensor_data, const bool& fast_domain)
{
    /*
     *  Safety disengage.
     */
    if (fabs(radiansToDegrees(sensor_data.attitude_y)) >= 20)
    {
        /*
         *  The controller becomes inactive if the magnitude of the
         *  Y attitude (pitch) is greater than 20 degrees.
         */
        active_ = false;
    }
    else
    {
        /*
         *  If the controller was inactive and is now becoming active,
         *  reset all integrated errors.
         */
        if (!active_)
        {
            pid_controller_position_x_.resetErrorIntegral();
            pid_controller_attitude_y_.resetErrorIntegral();
            pid_controller_attitude_z_.resetErrorIntegral();
        }

        /*
         *  The controller is active if the magnitude of the
         *  Y attitude (pitch) is within 20 degrees.
         */
        active_ = true;
    }
    // logger_->logDebug("active: "+String(active_));

    /*
     *  The reason behind the fast and slow time domain here is
     *  similar to the explanation in the sensor class.
     *  Controlling Y attitude (pitch) requires little to no
     *  delay and thus requires fast domain. Controlling X position
     *  (forward/backward) and Z attitude (yaw), however, the slow
     *  domain is more suitable as their sensor data is noisier.
     */
    if (fast_domain)
    {
        /*
         *  Set the plant state input (Y) of the Y attitude (pitch)
         *  PID controller to be the Y attitude (pitch) in the given
         *  sensor data struct from the function parameter.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_attitude_y_.setState(sensor_data.attitude_y);

        /*
         *  Set the error derivative input (delta e) of the Y attitude
         *  (pitch) PID controller to be the Y angular velocity in the
         *  given sensor data struct from the function parameter.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_attitude_y_.setErrorDifferential(sensor_data.angular_velocity_y);

        /*
         *  Execute the Y attitude (pitch) PID controller and store
         *  the output into the member Y attitude (pitch) PID
         *  controller output variable.
         */
        // TODO PART 2 YOUR CODE HERE.
        output_attitude_y_ = pid_controller_attitude_y_.control();
    }
    else
    {
        /*
         *  Set the plant state input (Y) of the X position
         *  (forward/backward) PID controller to be the X
         *  position in the given sensor data struct from
         *  the function parameter.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_position_x_.setState(sensor_data.position_x);

        /*
         *  Set the error derivative input (delta e) of the X position
         *  (forward/backward) PID controller to be the X linear velocity
         *  in the given sensor data struct from the function parameter.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_position_x_.setErrorDifferential(sensor_data.velocity_x);

        /*
         *  Set the open loop input of the Z attitude (yaw) PID controller
         *  to be the negative product between the Z attitude (yaw) PID
         *  controller target (R) and the magnitude of the X linear velocity
         *  in the given sensor data struct from the function parameter.
         *
         *  The reason behind such a formulation is that the robot cannot
         *  turn well on low speeds since the yaw control is merely a bias
         *  between the two motors. Therefore, the open loop input is made
         *  to be proportional to the magnitude of the X linear velocity so
         *  that the yaw control only provide bias at considerable X linear
         *  velocity.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_attitude_z_.setOpenLoop(-pid_controller_attitude_z_.getTarget()*sensor_data.velocity_x);

        /*
         *  Set the error derivative input (delta e) of the Z attitude (yaw)
         *  PID controller to be the Z angular velocity in the given sensor
         *  data struct from the function parameter.
         */
        // TODO PART 2 YOUR CODE HERE.
        pid_controller_attitude_z_.setErrorDifferential(sensor_data.angular_velocity_z);

        /*
         *  Execute the X position (forward/backward) PID controller
         *  and store the output into the member X position
         *  (forward/backward) PID controller output variable.
         */
        // TODO PART 2 YOUR CODE HERE.
        //output_position_x_ = pid_controller_position_x_.control();

        /*
         *  Execute the Z attitude (yaw) PID controller and store
         *  the output into the member Z attitude (yaw) PID
         *  controller output variable.
         */
        // TODO PART 2 YOUR CODE HERE.
        //output_attitude_z_ = pid_controller_attitude_z_.control();
    }

    /*
     *  Produce the left motor output by adding the
     *  X position PID controller output with the
     *  Y attitude PID controller output and then subtract
     *  the Z attitude PID controller output.
     */
    // TODO PART 2 YOUR CODE HERE.
    float left_motor = output_position_x_+output_attitude_y_-output_attitude_z_;

    /*
     *  Produce the right motor output by adding all three
     *  PID controller outputs.
     */
    // TODO PART 2 YOUR CODE HERE.
    float right_motor = output_position_x_+output_attitude_y_+output_attitude_z_;

    if (!active_)
    {
        /*
        *  Stop the motors by setting the motor outputs
        *  to 0 if the controller is inactive.
        */
        // TODO PART 2 YOUR CODE HERE.
        left_motor = 0;
        right_motor = 0;
    }

    /*
     *  Set the motor enable in the member actuation
     *  command struct to true, enabling the motors.
     */
    // TODO PART 2 YOUR CODE HERE.
    actuation_command_.motor_enable = true;

    /*
     *  Set the motor directions in the member actuation
     *  command struct to be the sign of the motor output
     *  values. For example, the left motor forward flag
     *  should be true if the left motor output value is
     *  positive or zero.
     */
    // TODO PART 2 YOUR CODE HERE.
    actuation_command_.motor_left_forward = (left_motor>=0) ? true : false;
    actuation_command_.motor_right_forward = (right_motor>=0) ? true : false;

    /*
     *  Take the absolute value, using the fabs function,
     *  of the motor output values, and constain, using
     *  the Arduino constrain function, the magnitude to be
     *  within the minimum and the maximum motor PWM values.
     *  See the Motor parameter enum. Remember to static_cast
     *  the Motor parameter enums to float before giving them
     *  to the Arduino constrain function.
     */
    // TODO PART 2 YOUR CODE HERE.
    //logger_->logDebug(String(left_motor));
    actuation_command_.motor_left_pwm = constrain(fabs(left_motor), float(Motor::pwm_min), float(Motor::pwm_max));
    // logger_->logDebug(String(actuation_command_.motor_left_pwm));

    actuation_command_.motor_right_pwm = constrain(fabs(right_motor), float(Motor::pwm_min), float(Motor::pwm_max));
}
}   // namespace tumbller