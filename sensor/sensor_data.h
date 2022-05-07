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
 *  @file   sensor_data.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor data header.
 *
 *  This file defines sensor data classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef SRC_SENSOR_SENSOR_DATA_H_
#define SRC_SENSOR_SENSOR_DATA_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Sensor data struct.
 *
 *  This struct contains sensor data entries,
 *  such as position, attitude, linear velocity,
 *  angular velocity, linear acceleration, and so on.
 *
 *  All spatial data is in the standard body reference frame.
 *
 *  Standard body reference frame:
 *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
 *
 *  Rotational right-hand rule:
 *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
 */
struct SensorData
{
    float position_x;   //!< In meters.
    float velocity_x;   //!< In meters per second.
    float acceleration_x;   //!< In meters per second squared.
    float acceleration_y;   //!< In meters per second squared.
    float acceleration_z;   //!< In meters per second squared.
    float attitude_x;   //!< Roll, in radians, unused.
    float attitude_y;   //!< Pitch, in radians.
    float attitude_z;   //!< Yaw, in radians, unused.
    float angular_velocity_x;   //!< Roll rate, in radians per second.
    float angular_velocity_y;   //!< Pitch rate, in radians per second.
    float angular_velocity_z;   //!< Yaw rate, in radians per second.
    float distance_ultrasound;  //!< In meters.
    float temperature;  //!< In Celsius.

    /**
     *  @brief  Sensor data struct constructor.
     *
     *  This constructor initializes all sensor data struct entries to 0.
     */
    SensorData() : position_x(0), velocity_x(0), acceleration_x(
            0), acceleration_y(0), acceleration_z(0), attitude_x(0), attitude_y(
            0), attitude_z(0), angular_velocity_x(0), angular_velocity_y(
            0), angular_velocity_z(0), distance_ultrasound(0), temperature(0)
    {
    }
};
}   // namespace tumbller

#endif  // SRC_SENSOR_SENSOR_DATA_H_