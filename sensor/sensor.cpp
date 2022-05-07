/*
 *  Copyright (C) 2022  University of Illinois Board of Trustees
 *
 *  Developed by:   Simon Yu (jundayu2@illinois.edu)
 *                  Department of Electrical and Computer Engineering
 *                  https://urldefense.com/v3/__https://www.simonyu.net/__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPEcyYt2p$ 
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
 *  along with this program.  If not, see <https://urldefense.com/v3/__https://www.gnu.org/licenses/__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPLD9wF_H$ >.
 */

/**
 *  @file   sensor.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class source.
 *
 *  This file implements the sensor class.
 */

/*
 *  External headers.
 */
#include <Adafruit_Sensor.h>

/*
 *  Project headers.
 */
#include "../actuator/actuator.h"
#include "../utility/logger.h"
#include "../utility/math.h"
#include "../common/pin.h"
#include "../sensor/sensor.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
Sensor::Sensor(Logger* logger, Actuator* actuator) : logger_(
        logger), actuator_(actuator), low_pass_filter_velocity_x_(
        logger), encoder_steps_total_left_(0), encoder_steps_total_right_(
        0), encoder_steps_slow_domain_left_(0), encoder_steps_slow_domain_right_(
        0), encoder_steps_fast_domain_left_(0), encoder_steps_fast_domain_right_(
        0), ultrasound_timer_(0), period_fast_domain_(
        0), period_slow_domain_(0)
{
    /*
     *  Initialize MPU and validate the initialization.
     */
    if (!mpu_.begin())
    {
        logger_->logError("Failed to initialize MPU");
        return;
    }

    /*
     *  Set pin mode for encoder and ultrasound pins.
     *  See the Pin enum for details.
     */
    // TODO PART 1 YOUR CODE HERE.
    pinMode(Pin::motor_left_encoder, INPUT);
    pinMode(Pin::motor_right_encoder, INPUT);
    pinMode(Pin::ultrasound_ping, OUTPUT);
    pinMode(Pin::ultrasound_pong, INPUT);

    /*
     *  Initialize ultrasound ping pin to LOW.
     */
    // TODO PART 1 YOUR CODE HERE.
    digitalWrite(Pin::ultrasound_ping, LOW);

    /*
     *  Configure MPU.
     */
    mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu_.setGyroRange(MPU6050_RANGE_250_DEG);

    /*
     *  Perform initial MPU read.
     */
    // TODO PART 1 YOUR CODE HERE.
    readMPU();

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO PART 1 YOUR CODE HERE.
    float attitude_y_raw = -atan2(sensor_data_.angular_velocity_x,sensor_data_.angular_velocity_z);
    sensor_data_.attitude_y = attitude_y_raw;

    /*
     *  Configure Y attitude Kalman filter.
     */
    kalman_filter_attitude_y_.setAngle(radiansToDegrees(sensor_data_.attitude_y));
    kalman_filter_attitude_y_.setQangle(0.001);
    kalman_filter_attitude_y_.setQbias(0.005);
    kalman_filter_attitude_y_.setRmeasure(0.5);

    /*
     *  Configure X velocity low-pass filter.
     */
    low_pass_filter_velocity_x_.setBeta(0.7);
}

SensorData
Sensor::getSensorData() const
{
    /*
     *  Return the member sensor data struct.
     */
    // TODO PART 1 YOUR CODE HERE.
    return sensor_data_;
}

void
Sensor::setPeriod(const float& period, const bool& fast_domain)
{
    if (fast_domain)
    {
        /*
         *  Set fast domain period.
         */
        // TODO PART 1 YOUR CODE HERE.
        period_fast_domain_=period;
    }
    else
    {
        /*
         *  Set slow domain period.
         */
        // TODO PART 1 YOUR CODE HERE.
        period_slow_domain_=period;
    }
}

void
Sensor::onEncoderLeftChange()
{
    /*
     *  Increment left fast domain encoder step counter.
     */
    // TODO PART 1 YOUR CODE HERE.
    encoder_steps_fast_domain_left_++;
}

void
Sensor::onEncoderRightChange()
{
    /*
     *  Increment right fast domain encoder step counter.
     */
    // TODO PART 1 YOUR CODE HERE.
    encoder_steps_fast_domain_right_++;
}

void
Sensor::onUltrasound(const bool& rising)
{
    if (rising)
    {
        /*
         *  Record the current time in microseconds into
         *  the ultrasound pulse wave timer, i.e., the time
         *  that the ultrasound pulse wave started to rise.
         */
        // TODO PART 1 YOUR CODE HERE.
         ultrasound_timer_=micros();
    }
    else
    {
        /*
         *  Calculate the difference (duration) between the
         *  current time and the time that the ultrasound
         *  pulse wave started to rise, i.e., the time width
         *  of the ultrasound pulse wave.
         */
        // TODO PART 1 YOUR CODE HERE.
        ultrasound_timer_=micros()-ultrasound_timer_;
    }
}

void
Sensor::sense(const bool& fast_domain)
{
    /*
     *  The fast domain tasks are the ones that requires
     *  faster execution frequency, such as the reading of
     *  the MPU, the attitude calculation, and etc. If such
     *  tasks were performed too seldomly, it might causes
     *  inaccurate or delayed data sampling. For example, if
     *  the attitude was calculated too seldomly, then it
     *  is possible that the plant has already moved to another
     *  pose during the gap between the calculations, which could
     *  cause, for instance, the robot failing to perfectly
     *  balance itself due to the delayed sample.
     *
     *  The slow domain tasks, on the other hand, are the ones
     *  that are better executed at lower frequency due to noise or
     *  performance issues. For example, the velocity here is
     *  calculated by dividing the encoder step count per period by the
     *  period. The smaller the period, the noisier the velocity.
     *  Additionally, it is unnecessary to read less important sensors
     *  such as the ultrasonic sensor at higher frequency. Thus, it's
     *  more performance-friendly to process such tasks in slow domain.
     */
    if (fast_domain)
    {
        /*
         *  Read MPU.
         */
        // TODO PART 1 YOUR CODE HERE.
        readMPU();

        /*
         *  Calculate attitude.
         */
        // TODO PART 1 YOUR CODE HERE.
        calculateAttitude();

        /*
         *  Read encoders.
         */
        // TODO PART 1 YOUR CODE HERE.
        readEncoder();

        /*
         *  Reset fast domain encoder step counters.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_fast_domain_left_=0;
        encoder_steps_fast_domain_right_=0;
    }
    else
    {
        /*
         *  Read ultrasonic sensor.
         */
        // TODO PART 1 YOUR CODE HERE.
        readUltrasound();

        /*
         *  Calculate velocity.
         */
        // TODO PART 1 YOUR CODE HERE.
        calculateVelocity();

        /*
         *  Reset encoder slow domain step counters.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_slow_domain_left_=0;
        encoder_steps_slow_domain_right_=0;
    }
}

void
Sensor::readMPU()
{
    /*
     *  Sensor event structs.
     *
     *  sensors_event_t struct reference:
     *  https://urldefense.com/v3/__http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPAg4eH4D$ 
     */
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    /*
     *  Read from MPU and populate sensor event structs.
     */
    if (!mpu_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        logger_->logError("Failed to read MPU");
        return;
    }

    /*
     *  Using the populated sensor event structs, populate
     *  the corresponding entries in the member sensor
     *  data struct.
     *
     *  Note that the raw data read from the MPU are not in the
     *  standard body reference frame. Refer to the following materials
     *  to correctly convert the raw data into the standard body
     *  reference frame.
     *
     *  Standard body reference frame:
     *  https://urldefense.com/v3/__https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPJwdAzlh$ 
     *
     *  Rotational right-hand rule:
     *  https://urldefense.com/v3/__https://en.wikipedia.org/wiki/Right-hand_rule*Rotations__;Iw!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPFfB8zwG$ 
     */
    // TODO PART 1 YOUR CODE HERE.

    sensor_data_.acceleration_x = acceleration.acceleration.x;
    sensor_data_.acceleration_y = acceleration.acceleration.y;
    sensor_data_.acceleration_z = acceleration.acceleration.z;

    sensor_data_.angular_velocity_x = angular_velocity.gyro.x;
    sensor_data_.angular_velocity_y = angular_velocity.gyro.y;
    sensor_data_.angular_velocity_z = angular_velocity.gyro.z;
    sensor_data_.temperature = temperature.temperature;
}

void
Sensor::readEncoder()
{
    /*
     *  Validate Actuator object pointer.
     */
    if (!actuator_)
    {
        logger_->logError("Actuator missing");
        return;
    }

    /*
     *  There are two encoders on each of the two motors.
     *  At least two encoders are needed for one motor for
     *  detecting the motor's spining direction. However,
     *  due to the limited number of pins that the Arduino
     *  board have, only one of the two encoders for both
     *  motors is connected. Therefore, it is currently not
     *  possible for us to actively detect the motor
     *  directions. However, we can do it passively, i.e.,
     *  incrementing or decrementing the encoder step counters
     *  depending on the motor directions that the controller
     *  produced to the actuator. For example, if the motor
     *  direction in the last actuation command was forward,
     *  then we increment the counters, otherwise decrement.
     *
     *  Incremental rotary encoder references:
     *  https://urldefense.com/v3/__https://www.seeedstudio.com/blog/2020/01/19/rotary-encoders-how-it-works-how-to-use-with-arduino/__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPBVn2O4X$ 
     */
    if (actuator_->getActuationCommand().motor_left_forward)
    {
        /*
         *  If the left motor is currently moving forward,
         *  add the fast domain encoder step counter
         *  towards the total step counter and the slow
         *  domain step counter.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_total_left_ += encoder_steps_fast_domain_left_;
        encoder_steps_slow_domain_left_ += encoder_steps_fast_domain_left_;
    }
    else
    {
        /*
         *  If the left motor is currently moving backward,
         *  subtract the fast domain encoder step counter
         *  towards the total step counter and the slow
         *  domain step counter.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_total_left_ -= encoder_steps_fast_domain_left_;
        encoder_steps_slow_domain_left_ -= encoder_steps_fast_domain_left_;
    }

    if (actuator_->getActuationCommand().motor_right_forward)
    {
        /*
         *  If the right motor is currently moving forward,
         *  add the fast domain encoder step counter
         *  towards the total step counter and the slow
         *  domain step counter.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_total_right_ += encoder_steps_fast_domain_right_;
        encoder_steps_slow_domain_right_ += encoder_steps_fast_domain_right_;
    }
    else
    {
        /*
         *  If the right motor is currently moving backward,
         *  subtract the fast domain encoder step counter
         *  towards the total step counter and the slow
         *  domain step counter.
         */
        // TODO PART 1 YOUR CODE HERE.
        encoder_steps_total_right_ -= encoder_steps_fast_domain_right_;
        encoder_steps_slow_domain_right_ -= encoder_steps_fast_domain_right_;
    }

    /*
     *  Take an average between the left and right total
     *  encoder step counters, convert the averaged total
     *  encoder steps into meters, and populate the
     *  corresponding entry in the member sensor data struct.
     *
     *  3700 encoder steps = 1 meter translational movement (from experiment.)
     */
    // TODO PART 1 YOUR CODE HERE.
    float avg = (encoder_steps_total_left_+encoder_steps_total_right_)/2.0;
    avg /=3700;
    sensor_data_.position_x = avg;
}

void
Sensor::readUltrasound()
{
    /*
     *  Convert the calculated ultrasound pulse wave duration
     *  into meters using the microsecondsToMeters function,
     *  and populate the corresponding entry in the member
     *  sensor data struct.
     */
    // TODO PART 1 YOUR CODE HERE.
    sensor_data_.distance_ultrasound = microsecondsToMeters(ultrasound_duration_,1);

    /*
     *  Emit a new ultrasound pulse wave by setting the
     *  ultrasound ping pin to HIGH for 10 microseconds,
     *  using the Arduino delayMicroseconds function for
     *  the delay, and then back to LOW.
     */
    // TODO PART 1 YOUR CODE HERE.
    digitalWrite(Pin::ultrasound_ping, HIGH);
    delayMicroseconds(10);
    digitalWrite(Pin::ultrasound_ping, LOW);
}

void
Sensor::calculateAttitude()
{
    /*
     *  Validate fast domain period.
     */
    if (period_fast_domain_ <= 0)
    {
        logger_->logError("Invalid period");
        return;
    }

    /*
     *  Calculate the raw Y attitude (pitch) data using
     *  the populated linear accelerations in the member
     *  sensor data struct. Refer to the following materials
     *  to correctly convert the calculated data into the
     *  standard body reference frame.
     *
     *  Note that the attitudes (roll, pitch, and yaw) are
     *  angles between two pairs of acceleration vectors. Use
     *  atan2 function instead of atan for correct signedness.
     *
     *  Remember to perform the same calculation in the
     *  constructor for initialization but populate the
     *  member sensor data struct using the raw data (unfiltered)
     *  directly, since the Kalman filter had not been initialized
     *  at that point in the constructor.
     *
     *  Standard body reference frame:
     *  https://urldefense.com/v3/__https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes__;!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPJwdAzlh$ 
     *
     *  Rotational right-hand rule:
     *  https://urldefense.com/v3/__https://en.wikipedia.org/wiki/Right-hand_rule*Rotations__;Iw!!DZ3fjg!uE2E9E1DRKClgwHilK7jrm346zMewqAOA2MCk56ba5wG92S-_50-eKL1i_UMPFfB8zwG$ 
     */
    // TODO PART 1 YOUR CODE HERE.
    float attitude_y_raw = -atan2(sensor_data_.angular_velocity_x,sensor_data_.angular_velocity_z);

    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    float attitude_y_kalman_filter = kalman_filter_attitude_y_.getAngle(radiansToDegrees(
            attitude_y_raw), radiansToDegrees(sensor_data_.angular_velocity_y), period_fast_domain_);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member sensor
     *  data struct.
     */
    // TODO PART 1 YOUR CODE HERE.
    sensor_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
}

void
Sensor::calculateVelocity()
{
    /*
     *  Take an average between the left and right slow domain
     *  encoder step counters, convert the averaged slow domain
     *  encoder steps into meters, divide the converted average
     *  with the slow domain period to get the raw X velocity,
     *  filter the raw X velocity using the low-pass filter, and
     *  then finally populate the corresponding entry in the
     *  member sensor data struct.
     *
     *  3700 encoder steps = 1 meter translational movement (from experiment.)
     */
    // TODO PART 1 YOUR CODE HERE.
    float avg = (encoder_steps_slow_domain_left_+encoder_steps_slow_domain_right_)/2.0;
    avg /= 3700;
    float x_raw_velocity = avg/period_slow_domain_;
    x_raw_velocity = low_pass_filter_velocity_x_.filter(x_raw_velocity);
    sensor_data_.velocity_x = x_raw_velocity;
}
}   // namespace tumbller
