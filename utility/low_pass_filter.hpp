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
 *  @file   low_pass_filter.hpp
 *  @author Simon Yu
 *  @date   01/19/2022
 *  @brief  Low-pass filter templated class header.
 *
 *  This file defines and implements the low-pass filter
 *  templated class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_UTILITY_LOW_PASS_FILTER_H_
#define SRC_UTILITY_LOW_PASS_FILTER_H_

/*
 *  Project headers.
 */
#include "../utility/logger.h"

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/**
 *  @tparam Type Type of data for filtering.
 *  @brief  Low-pass filter templated class.
 *
 *  This templated class provides functions for
 *  filtering data using a simple, discrete
 *  low-pass filter.
 */
template <typename Type>
class LowPassFilter
{
public:

    /**
     *  @param  logger Logger object pointer.
     *  @brief  Low-pass filter templated class constructor.
     *
     *  This constructor initializes all class member variables.
     */
    inline
    LowPassFilter(Logger* logger) : logger_(logger), data_filtered_(
            0), beta_(0)
    {
    }

    /**
     *  @param  beta Beta parameter.
     *  @brief  Set the Beta parameter.
     *
     *  This function sets the Beta parameter.
     */
    inline void
    setBeta(const float& beta)
    {
        /*
         *  Set the Beta parameter.
         */
        beta_ = beta;
    }

    /**
     *  @tparam Type Type of filtered data.
     *  @param  data New data.
     *  @return Templated filtered data.
     *  @brief  Filter the new data using a low-pass filter.
     *
     *  This function filters the new data
     *  using a simple, discrete low-pass filter.
     */
    inline Type
    filter(const Type& data)
    {
        /*
         *  Simple, discrete low-pass filter:
         *  https://en.wikipedia.org/wiki/Low-pass_filter#Difference_equation_through_discrete_time_sampling
         */
        data_filtered_ = beta_ * data_filtered_ + (1 - beta_) * data;

        /*
         *  Return filtered data.
         */
        return data_filtered_;
    }

private:

    Logger* logger_;    //!< Logger object pointer.
    Type data_filtered_;    //!< Templated filtered data.
    float beta_;    //!< Beta parameter.
};
}   // namespace tumbller

#endif  // SRC_UTILITY_LOW_PASS_FILTER_H_