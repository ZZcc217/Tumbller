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
 *  @file   pid_controller.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  PIDController class header.
 *
 *  This file defines the PID controller class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_CONTROLLER_PID_CONTROLLER_H_
#define SRC_CONTROLLER_PID_CONTROLLER_H_

/*
 *  Project headers.
 */
#include "../common/motor.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Logger;

/**
 *  @brief  PID controller class.
 *
 *  This class provides functions for establishing
 *  a PID control system, which aims to generate
 *  an output that minimizes the error between the
 *  plant state input and a target in a closed loop.
 *
 *  See Lecture 13 slide 17 for the definition of
 *  the PID controller.
 */
class PIDController
{
public:

    /**
     *  @brief  PID controller gain struct.
     *
     *  This struct contains PID controller gain entries,
     *  such as the proportional, integral, differential
     *  gains, and so on.
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    struct Gain
    {
        float proportional; //!< Proportional gain (Kp).
        float integral; //!< Integral gain (Ki).
        float differential; //!< Differential gain (Kd).
        float open_loop;    //!< Open loop gain.
        float integral_max; //!< Maximum integrated error.

        /**
         *  @brief  PID controller gain struct constructor.
         *
         *  This constructor initializes all PID controller gain entries to 0.
         */
        Gain() : proportional(0), integral(0), differential(0), open_loop(
                0), integral_max(0)
        {
        }
    };

    /**
     *  @brief  PID controller saturation struct.
     *
     *  This struct contains PID controller saturation entries,
     *  such as the input and output saturations.
     */
    struct Saturation
    {
        float saturation_input_upper;   //!< Input saturation upper bound.
        float saturation_input_lower;   //!< Input saturation lower bound.
        float saturation_output_upper;  //!< Output saturation upper bound.
        float saturation_output_lower;  //!< Output saturation lower bound.

        /**
         *  @brief  PID controller saturation struct constructor.
         *
         *  This constructor initializes all PID controller saturation
         *  entries to their respective extremes.
         */
        Saturation() : saturation_input_upper(3.4028235E+38), saturation_input_lower(
                -1 * 3.4028235E+38), saturation_output_upper(
                3.4028235E+38), saturation_output_lower(-1 * 3.4028235E+38)
        {
        }
    };

    /**
     *  @param  logger Logger object pointer.
     *  @brief  PID controller class constructor.
     *
     *  This constructor initializes all class member variables.
     */
    PIDController(Logger* logger);

    /**
     *  @return Target (R).
     *  @brief  Get the target (R).
     *
     *  This function returns the target (R).
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    float
    getTarget() const;

    /**
     *  @param  gain PID controller gain struct.
     *  @brief  Set the PID controller gain.
     *
     *  This function sets the PID controller gain
     *  and resets the integrated error.
     */
    void
    setGain(const Gain& gain);

    /**
     *  @param  saturation PID controller saturation struct.
     *  @brief  Set the PID controller saturation.
     *
     *  This function sets the PID controller saturation.
     */
    void
    setSaturation(const Saturation& saturation);

    /**
     *  @param  state Plant state input (Y).
     *  @brief  Set the Plant state input (Y).
     *
     *  This function sets the plant state input (Y).
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    void
    setState(const float& state);

    /**
     *  @param  target Target (R).
     *  @brief  Set the target (R).
     *
     *  This function sets the target (R).
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    void
    setTarget(const float& target);

    /**
     *  @param  period Control period, in seconds.
     *  @brief  Set the control period of the PID controller.
     *
     *  This function sets the control period of the PID controller.
     */
    void
    setPeriod(const float& period);

    /**
     *  @param  open_loop Open loop input.
     *  @brief  Set the open loop input.
     *
     *  This function sets the open loop input.
     */
    void
    setOpenLoop(const float& open_loop);

    /**
     *  @param  error_differential Error derivative input (delta e).
     *  @brief  Set the error derivative input (delta e).
     *
     *  This function sets the error derivative input (delta e).
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    void
    setErrorDifferential(const float& error_differential);

    /**
     *  @brief  Reset the integrated error (integral of e).
     *
     *  This function resets the integrated error (integral of e).
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    void
    resetErrorIntegral();

    /**
     *  @return Output (u).
     *  @brief  Execute the PID control system.
     *
     *  This function executes the PID control system, which generates control
     *  output (u). This function is expected to be called periodically.
     *
     *  See Lecture 13 slide 17 for the variable definitions.
     */
    float
    control();

private:

    Logger* logger_;    //!< Logger object pointer.
    Gain gain_; //!< PID controller gain struct.
    Saturation saturation_; //!< PID controller saturation struct.
    float state_;   //!< Plant state input (Y).
    float target_;  //!< Target (R).
    float period_;  //!< Control period, in seconds.
    float open_loop_;    //!< Open loop input.
    float error_differential_;  //!< Error derivative input (delta e).
    float error_integral_;  //!< Integrated error (integral of e).
};
}   // namespace tumbller

#endif  // SRC_CONTROLLER_PID_CONTROLLER_H_
