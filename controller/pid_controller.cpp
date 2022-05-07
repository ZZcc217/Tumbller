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
 *  @file   pid_controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  PIDController class source.
 *
 *  This file implements the PID controller class.
 */

/*
 *  Project headers.
 */
#include "../utility/logger.h"
#include "../common/motor.h"
#include "../controller/pid_controller.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
PIDController::PIDController(Logger* logger) : logger_(logger), state_(0), target_(
        0), period_(0), error_differential_(0), error_integral_(0)
{
}

float
PIDController::getTarget() const
{
    /*
     *  Return the target (R).
     */
    // TODO PART 2 YOUR CODE HERE.
    return target_;
}

void
PIDController::setGain(const Gain& gain)
{
    /*
     *  Set the PID controller gain.
     */
    // TODO PART 2 YOUR CODE HERE.
    gain_ = gain;

    /*
     *  The integrated error would become meaningless
     *  with new gains. Reset the integrated error
     *  (integral of e).
     */
    // TODO PART 2 YOUR CODE HERE.
    error_integral_ = 0;
}

void
PIDController::setSaturation(const Saturation& saturation)
{
    /*
     *  Set the PID controller saturation.
     */
    // TODO PART 2 YOUR CODE HERE.
    saturation_ = saturation;
}

void
PIDController::setState(const float& state)
{
    /*
     *  Set the plant state input (Y).
     */
    // TODO PART 2 YOUR CODE HERE.
    state_ = state;
}

void
PIDController::setTarget(const float& target)
{
    /*
     *  Set the target (R).
     */
    // TODO PART 2 YOUR CODE HERE.
    target_ = target;

    /*
     *  The integrated error would become meaningless
     *  with a new target. Reset the integrated error
     *  (integral of e).
     */
    // TODO PART 2 YOUR CODE HERE.
    error_integral_ = 0;
}

void
PIDController::setPeriod(const float& period)
{
    /*
     *  Set the control period.
     */
    // TODO PART 2 YOUR CODE HERE.
    period_ = period;

    /*
     *  The integrated error would become meaningless
     *  with a new control period. Reset the integrated
     *  error (integral of e).
     */
    // TODO PART 2 YOUR CODE HERE.
    error_integral_ = 0;
}

void
PIDController::setOpenLoop(const float& open_loop)
{
    /*
     *  Set the open loop input.
     *
     *  Open loop control:
     *  https://urldefense.com/v3/__https://en.wikipedia.org/wiki/Feed_forward_(control)media/File:Control_Systems.png__;Iw!!DZ3fjg!s3kgEcPI6nVTJcr3Rk_6dxTTkYb69bmU-b6AHbdKAa2F7t9QbgRaEOsJnGOvCE_6hVjV$ 
     */
    // TODO PART 2 YOUR CODE HERE.
    open_loop_ = open_loop;
}

void
PIDController::setErrorDifferential(const float& error_differential)
{
    /*
     *  Set the error derivative input (delta e).
     */
    // TODO PART 2 YOUR CODE HERE.
    error_differential_ = error_differential;
}

void
PIDController::resetErrorIntegral()
{
    /*
     *  Reset the integrated error (integral of e) to 0.
     */
    // TODO PART 2 YOUR CODE HERE.
    error_integral_ = 0;
}

float
PIDController::control()
{
    /*
     *  Validate control period.
     */
    if (period_ <= 0)
    {
        logger_->logError("Invalid period");
        return 0;
    }

    /*
     *  Calculate the error (e) between the plant input state (Y)
     *  and target (R), and constrain, using the Arduino constrain
     *  function, the calculated error between the input saturation
     *  upper and lower bounds.
     *
     *  For the "correct" signedness, calculate the error (e) by
     *  subtracting state with target (Y - R). Otherwise, the signs
     *  of your PID controller gains may be flipped.
     *
     *  See Lecture 13 slide 17 for the definition of
     *  the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    float error = state_-target_;
    // logger_->logDebug("Error is "+String(error));
    error = constrain(error, saturation_.saturation_input_lower, saturation_.saturation_input_upper);

    /*
     *  Calculate the new discrete integral of error (integral of e),
     *  and constrain, using the Arduino constrain function, the
     *  calculated integral of error (integral of e) between the
     *  negative and positive maximum integrated error from the
     *  PID controller gain struct.
     *
     *  The new discrete integrated error is the current integrated
     *  error plus the product between the control period and the
     *  current error (e).
     *
     *  The above is essentially the Riemann sum:
     *  https://urldefense.com/v3/__https://en.wikipedia.org/wiki/Riemann_sum__;!!DZ3fjg!s3kgEcPI6nVTJcr3Rk_6dxTTkYb69bmU-b6AHbdKAa2F7t9QbgRaEOsJnGOvCJLEOFQ_$ 
     *
     *  See Lecture 13 slide 17 for the definition of
     *  the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    error_integral_ += period_*error;
    error_integral_ = constrain(error_integral_, -gain_.integral_max, gain_.integral_max);
    //error_differential_ = error/period_;

    /*
     *  Calculate the proportional output.
     *
     *  See Lecture 13 slide 17 for the definition of
     *  the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    float proportional = gain_.proportional*error;

    /*
     *  Calculate the integral output.
     *
     *  See Lecture 13 slide 17 for the definition of
     *  the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    float integral = gain_.integral*error_integral_;

    /*
     *  Calculate the differential output.
     *
     *  See Lecture 13 slide 17 for the definition of
     *  the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    float differantial = gain_.differential*error_differential_;

    /*
     *  Calculate the open loop output.
     *
     *  The open loop output is the product between
     *  the open loop gain and the open loop input.
     *
     *  Open loop control:
     *  https://urldefense.com/v3/__https://en.wikipedia.org/wiki/Feed_forward_(control)media/File:Control_Systems.png__;Iw!!DZ3fjg!s3kgEcPI6nVTJcr3Rk_6dxTTkYb69bmU-b6AHbdKAa2F7t9QbgRaEOsJnGOvCE_6hVjV$ 
     */
    // TODO PART 2 YOUR CODE HERE.
    float open = gain_.open_loop*open_loop_;

    /*
     *  Calculate the sum of all the above four outputs
     *  and constrain, using the Arduino constrain function,
     *  the sum between the output saturation upper and
     *  lower bounds.
     */
    // TODO PART 2 YOUR CODE HERE.
    float sum = proportional+differantial+integral+open;
    
    // sum = constrain(sum, saturation_.saturation_output_lower, saturation_.saturation_output_upper);
    // logger_->logDebug(String(error)+" "+String(sum)+" ");

    /*
     *  Return the constrained sum of outputs as the
     *  final output of the PID controller.
     */
    // TODO PART 2 YOUR CODE HERE.
    return sum;
}
}   // namespace tumbller