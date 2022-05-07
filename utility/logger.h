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
 *  @file   logger.h
 *  @author Simon Yu
 *  @date   11/19/2021
 *  @brief  Logger class header.
 *
 *  This file defines the logger class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_UTILITY_LOGGER_H_
#define SRC_UTILITY_LOGGER_H_

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @brief  Logger class.
 *
 *  This class provides functions for
 *  logging to the serial with log levels.
 */
class Logger
{
public:

    /**
     *  @brief  Log level enum class.
     *
     *  This enum class contains log levels.
     */
    enum class LogLevel : int
    {
        error = 0,  //!< Errors.
        warn,   //!< Warnings.
        info,   //!< Information.
        debug   //!< Debugging.
    };

    /**
     *  @brief  Logger class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Addtionally, the constructor initializes the platform
     *  serial connection at the baud rate defined in the
     *  PlatformSerial enum.
     */
    Logger();

    /**
     *  @return Log level enum object.
     *  @brief  Get the logger status.
     *
     *  This function returns the worst log level ever occurred
     *  (only either warn or error.)
     */
    LogLevel
    getStatus() const;

    /**
     *  @param  log_level Log level enum object.
     *  @brief  Set the log level.
     *
     *  This function sets the log level.
     */
    void
    setLogLevel(const LogLevel &log_level);

    /**
     *  @param  log Log message string.
     *  @brief  Log raw message.
     *
     *  This function logs the given message as is.
     *  The function always prints a new line.
     *  The message is logged regardless of the log level.
     */
    void
    logRaw(const String &log);

    /**
     *  @param  log Log message string.
     *  @brief  Log error message.
     *
     *  This function logs the message in error format.
     *  A period is automatically appended to the message.
     *  The function always prints a new line.
     *  The message is logged regardless of the log level.
     */
    void
    logError(const String &log);

    /**
     *  @param  log Log message string.
     *  @brief  Log warning message.
     *
     *  This function logs the message in warning format.
     *  A period is automatically appended to the message.
     *  The function always prints a new line.
     *  The message is logged only if the log level is
     *  warn or better.
     */
    void
    logWarn(const String &log);

    /**
     *  @param  log Log message string.
     *  @brief  Log information message.
     *
     *  This function logs the message in information format.
     *  A period is automatically appended to the message.
     *  The function always prints a new line.
     *  The message is logged only if the log level is
     *  info or better.
     */
    void
    logInfo(const String &log);

    /**
     *  @param  log Log message string.
     *  @brief  Log debugging message.
     *
     *  This function logs the message in debugging format.
     *  A period is automatically appended to the message.
     *  The function always prints a new line.
     *  The message is logged only if the log level is
     *  debug.
     */
    void
    logDebug(const String &log);

private:

    LogLevel log_level_;    //!< Log level enum object.
    LogLevel status_;   //!< Status log level enum object.
};
}   // namespace tumbller

#endif  // SRC_UTILITY_LOGGER_H_