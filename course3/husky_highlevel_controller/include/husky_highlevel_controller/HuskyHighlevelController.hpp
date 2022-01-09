/* Copyright (C) 2019-2020 hnqiu. All Rights Reserved.
 * Licensed under the BSD-3-Clause license. See LICENSE for details.
 */


#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <cmath>

namespace husky_highlevel_controller {

/* @brief: Husky Highlevel Controller */
class HuskyHighlevelController {
public:
    /* @brief: Constructor */
    HuskyHighlevelController(ros::NodeHandle& nh);

    /* @brief: Copy constructor
     * defined as DELETED for simplicity 
     */
    HuskyHighlevelController(const HuskyHighlevelController &) = delete;
    /* @brief: no assignment = */
    HuskyHighlevelController& operator=(const HuskyHighlevelController &) = delete;

    /* @brief: Destructor */
    ~HuskyHighlevelController() = default;

    /* @brief: set Husky velocity
     * set robot's linear & angular velocity
     */
    void setVel(const float &vel, const std::string &dof);

    /* @brief: ROS topic publish function 
     * publish a message to topic /cmd_vel to send a Twist to the robot
     */
    void DriveHusky();

    /* @brief: adjust robot forward speed
     * adjust speed using saturated P control
     */
    void adjustSpeed(const float &dist);

    /* @brief: adjust robot heading
     * adjust heading using P control
     */
    void adjustHeading(const float &ang);

    /* @brief: visualize pillar with marker in RViz */
    void vizPillar();

private:
    /* data */
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;
    ros::Publisher vel_pub, viz_pub;
    geometry_msgs::Twist msg;
    visualization_msgs::Marker marker;
    float p_ang, p_vel;
    float pillar_pos[2];

    /* @brief: ROS topic callback function 
     * print out the position of the pillar with respect to the robot
     * and adjust the robot heading towards the pillar
     */
    void LaserCallback(const sensor_msgs::LaserScan &msg);

    /* @brief: initialize pillar marker in RViz */
    void initPillarMarker();
};

} /* namespace */
