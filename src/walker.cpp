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
* @file walk.cpp
* @author Jerrar Bukhari
* @date 16 November 2018
* @copyright 2018 Jerrar Bukhari
* @brief This file defines the methods for class "avoider" to make the robot
* go straight or turn
*/

#include <cmath>
#include <algorithm>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "walker/Avoider.h"

// global variable to store minimum front distance
float frontDist;

/**
 * @brief      Callabck function for subcriber to Laser scanner
 *
 * @param[in]  points  The range points from Laser scanner
 */
void depthCallback(const sensor_msgs::LaserScanConstPtr& points) {
    // arbitrary max value
    float min = 100;
    // custom min function to get closest point
    for (auto i : points->ranges) {
        if (!std::isnan(i)) {
            if (i < min) { min = i;}
        }
    }
    ROS_DEBUG("Distance: %f", min);
    frontDist = min;
}

/**
 * @brief      Main fucntion to facilitate sub and pub for turtlebot
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     returns 0 for succesful completion
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "walk");
    ros::NodeHandle n;
    // create subscriber to laser scanner data at /scan topic
    ros::Subscriber depth_sub = n.subscribe("/scan", 1,
                                                depthCallback);
    // advertise publisher to turtlebot veocity commands at given topic
    ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>
                        ("/mobile_base/commands/velocity", 5);
    // setup a fixed rate to publish topics
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel;
    // instantiate Avoider object to get turtlebot velocities
    Avoider avoidObstacle;
    while (ros::ok()) {
        vel = avoidObstacle.get_vel(frontDist);
        loop_rate.sleep();
        move_pub.publish(vel);
        ros::spinOnce();
    }

    return 0;
}


