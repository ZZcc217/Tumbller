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
 *  @file   sensor.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class header.
 *
 *  This file defines the sensor class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

/*
 *  External headers.
 */
#include <Adafruit_MPU6050.h>
#include <Kalman.h>

/*
 *  Project headers.
 */
#include "../utility/low_pass_filter.hpp"
#include "../sensor/sensor_data.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Actuator;
class Logger;

/**
 *  @brief  Sensor class.
 *
 *  This class provides functions for retrieving data
 *  from various sensors, such as the motion processing
 *  unit (MPU), the motor encoders, and the ultrasonic
 *  sensor, and for populating data into the sensor data
 *  struct. The class also provides callback functions
 *  for interrupt-based sensors, such as the encoders
 *  and the ultrasonic sensor.
 */
class Sensor
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @param  actuator Actuator object pointer.
     *  @brief  Sensor class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor initializes the related Arduino
     *  I/O pins as well as the MPU and the ultrasonic sensor.
     */
    Sensor(Logger* logger, Actuator* actuator);

    /**
     *  @return Sensor data struct.
     *  @brief  Get the class member sensor data strut.
     *
     *  This function returns the class member sensor data strut.
     */
    SensorData
    getSensorData() const;

    /**
     *  @param  period Time period, in seconds.
     *  @param  fast_domain Whether the given time period is for fast domain.
     *  @brief  Set the time or sampling period.
     *
     *  This function sets the time or sampling period, for fast or slow
     *  domain, for the sensor data acquisition process.
     */
    void
    setPeriod(const float& period, const bool& fast_domain);

    /**
     *  @brief  Left encoder callback function.
     *
     *  This function processes the interrupt from the left motor encoder.
     */
    void
    onEncoderLeftChange();

    /**
     *  @brief  Right encoder callback function.
     *
     *  This function processes the interrupt from the right motor encoder.
     */
    void
    onEncoderRightChange();

    /**
     *  @param  rising Whether the ultrasound pulse wave is rising.
     *  @brief  Ultrasonic sensor callback function.
     *
     *  This function processes the interrupt from the ultrasonic sensor.
     */
    void
    onUltrasound(const bool& rising);

    /**
     *  @param  fast_domain Whether to perform fast domain sensing.
     *  @brief  Sensor data acquisition function.
     *
     *  This function performs acquisition of all sensor data, for
     *  fast or slow domain, and populates the member sensor
     *  data struct. This function is expected to be called
     *  periodically.
     */
    void
    sense(const bool& fast_domain);

private:

    /**
     *  @brief  MPU reading function.
     *
     *  This function reads sensor data from the MPU and
     *  populates the corresponding entries in the member
     *  sensor data struct.
     */
    void
    readMPU();

    /**
     *  @brief  Encoder reading function.
     *
     *  This function reads sensor data from the motor
     *  encoders and populates the corresponding entries
     *  in the member sensor data struct.
     */
    void
    readEncoder();

    /**
     *  @brief  Ultrasonic sensor reading function.
     *
     *  This function reads sensor data from the ultrasonic
     *  sensor and populates the corresponding entries
     *  in the member sensor data struct.
     */
    void
    readUltrasound();

    /**
     *  @brief  Attitude calculation function.
     *
     *  This function calculates attitude data from acceleration
     *  data and populates the corresponding entries in the member
     *  sensor data struct. The function also filters certain data
     *  using the Kalman filter.
     */
    void
    calculateAttitude();

    /**
     *  @brief  Velocity calculation function.
     *
     *  This function calculates linear velocity data from encoder
     *  data and populates the corresponding entries in the member
     *  sensor data struct. The function also filters certain data
     *  using the low pass filter.
     */
    void
    calculateVelocity();

    Logger* logger_;    //!< Logger object pointer.
    Actuator* actuator_;    //!< Actuator object pointer.
    Adafruit_MPU6050 mpu_;  //!< Adafruit MPU6050 object.
    SensorData sensor_data_;    //!< Sensor data struct.
    Kalman kalman_filter_attitude_y_;   //!< Y attitude (pitch) Kalman filter object.
    LowPassFilter<float> low_pass_filter_velocity_x_;   //!< X velocity low-pass filter object.
    long encoder_steps_total_left_;    //!< Left encoder total step counter.
    long encoder_steps_total_right_;   //!< Right encoder total step counter.
    long encoder_steps_slow_domain_left_;   //!< Left encoder step counter per slow domain period.
    long encoder_steps_slow_domain_right_;  //!< Right encoder step counter per slow domain period.
    long encoder_steps_fast_domain_left_;  //!< Left encoder step counter per fast domain period.
    long encoder_steps_fast_domain_right_; //!< Right encoder step counter per fast domain period.
    unsigned long ultrasound_timer_;    //!< Ultrasound pulse wave timer, in microseconds.
    unsigned long ultrasound_duration_; //!< Ultrasound pulse wave duration (width), in microseconds.
    float period_fast_domain_;  //!< Fast domain time period, in seconds.
    float period_slow_domain_;  //!< Slow domain time period, in seconds.
};
}   // namespace tumbller

#endif  // SRC_SENSOR_SENSOR_H_
