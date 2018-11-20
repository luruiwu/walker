/**
 * Copyright (C) 2018  Jerrar Bukhari 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
* @file Avoider.h
* @author Jerrar Bukhari
* @date 16 November 2018
* @copyright 2018 Jerrar Bukhari
* @brief This file defines the methods for class "avoider" to make the robot
* go straight or turn
*/

#ifndef INCLUDE_WALKER_AVOIDER_H_
#define INCLUDE_WALKER_AVOIDER_H_

#include "geometry_msgs/Twist.h"

class Avoider {
 private:
    geometry_msgs::Twist straight;
    geometry_msgs::Twist turn;
    float thresh;
 public:
    geometry_msgs::Twist get_vel(float dist);
    Avoider();
};

#endif  // INCLUDE_WALKER_AVOIDER_H_
