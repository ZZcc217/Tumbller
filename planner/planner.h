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
 *  @file   planner.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Planner abstract class header.
 *
 *  This file defines the planner abstract class.
 */

/*
 *  Include guard.
 */
#ifndef SRC_PLANNER_PLANNER_H_
#define SRC_PLANNER_PLANNER_H_

/*
 *  tumbller namespace.
 */
namespace tumbller
{
/*
 *  Forward declaration.
 */
class Controller;
class Logger;

/**
 *  @brief  Planner abstract class.
 *
 *  This abstract class provides pure virtual functions
 *  for various types of planners to inherit and implement.
 */
class Planner
{
public:

    /**
     *  @brief  Planner abstract class pure virtual destructor.
     *
     *  This destructor is pure virtual and prohibits the
     *  instantiation of this abstract class.
     */
    virtual
    ~Planner() = default;

    /**
     *  @brief  Execute plan.
     *
     *  This function is pure virtual and its functionality
     *  is to be implemented by any child class.
     */
    virtual void
    plan() = 0;
};
}   // namespace tumbller

#endif  // SRC_PLANNER_PLANNER_H_