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
* @file Avoider.cpp
* @author Jerrar Bukhari
* @date 16 November 2018
* @copyright 2018 Jerrar Bukhari
* @brief This file defines the methods for class "avoider" to make the robot
* go straight or turn
*/

#include "walker/Avoider.h"


/**
 * @brief      Constructs the object Avoider.
 */
Avoider::Avoider() {
    // Initialize straight member to go straight
    straight.linear.x = 0.3;
    straight.linear.y = 0.0;
    straight.linear.z = 0.0;
    straight.angular.x = 0.0;
    straight.angular.y = 0.0;
    straight.angular.z = 0.0;

    // Initialize turn member to change direction
    turn.linear.x = 0.0;
    turn.linear.y = 0.0;
    turn.linear.z = 0.0;
    turn.angular.x = 0.0;
    turn.angular.y = 0.0;
    turn.angular.z = 0.5;

    thresh = 0.8;
}


/**
 * @brief      Gets the velocity for going straight or 
 * 				turning depending on the min value.
 *
 * @param[in]  min   The minimum distance detected by LaserScanner
 *
 * @return     The velocity in the from of Twist msg.
 */
geometry_msgs::Twist Avoider::get_vel(float min) {
    if (min < thresh) {
        return turn;
    } else {
        return straight;
    }
}
