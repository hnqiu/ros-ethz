/* Copyright (C) 2019-2020 hnqiu. All Rights Reserved.
 * Licensed under the BSD-3-Clause license. See LICENSE for details.
 */


#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


namespace husky_highlevel_controller {

/* @brief: Constructor */
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nh)
    : nodeHandle(nh), subscriber(), vel_pub(), viz_pub(), marker()
    , msg(), ctrl_p(.0) {
    // get param from config file
    nodeHandle.getParam("controller_gain", ctrl_p);
    std::string topic;
    int queue_size;
    if ( !nodeHandle.getParam("subscriber_topic", topic) 
        || !nodeHandle.getParam("queue_size", queue_size) ) 
    {
        ROS_ERROR("Could not find subscriber params!"); 
        ros::requestShutdown();
    }
    // create subscriber
    subscriber = nodeHandle.subscribe(topic, queue_size, 
        &HuskyHighlevelController::LaserCallback, this);
    // create publishers
    vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    viz_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    // pillar marker in RViz
    initPillarMarker();
    ROS_INFO("Husk highlevel controller node launched!");
}


/* @brief: set Husky velocity
 * set robot's linear & angular velocity
 */
void HuskyHighlevelController::setVel(const float &vel, const std::string &dof) {
    if (dof == "forward") {
        msg.linear.x = vel;
    }
    else if (dof == "ang") {
        msg.angular.z = vel;
    }
}


/* @brief: ROS topic publish function 
 * publish a message to topic /cmd_vel to send a Twist to the robot
 */
void HuskyHighlevelController::DriveHusky() {
    vel_pub.publish(msg);
}


/* @brief: adjust robot heading
 * adjust heading using P control
 */
void HuskyHighlevelController::adjustHeading(const float &ang) {
    float diff = -ang;
    setVel(ctrl_p * diff, "ang");
}


/* @brief: visualize pillar with marker in RViz */
void HuskyHighlevelController::vizPillar() {
    marker.pose.position.x = pillar_pos[0];
    marker.pose.position.y = pillar_pos[1];
    marker.pose.position.z = -1.0;
    viz_pub.publish(marker);
}


/*@brief: ROS topic callback function
 * print out the position of the pillar with respect to the robot
 * and adjust the robot heading towards the pillar
 */
void HuskyHighlevelController::LaserCallback(const sensor_msgs::LaserScan &msg) {
    // fist get the distance
    // typeof(msg.ranges) vector<float>(720)
    auto dist = std::min_element(msg.ranges.cbegin(), msg.ranges.cend());
    // then get the sensor angle
    // angle range [-135 deg, 135 deg]
    int count = dist - msg.ranges.cbegin();
    auto ang = msg.angle_min + msg.angle_increment * count;
    ROS_INFO_STREAM("Pillar is " << *dist << "m away at "
                    << ang / M_PI * 180.0 << " degrees");
    // calculate the coordinate
    pillar_pos[0] = *dist * std::cos(ang);
    pillar_pos[1] = *dist * std::sin(ang);
    ROS_INFO_STREAM("Pillar's coordinate to Husky is [" << pillar_pos[0]
                    << ", " << pillar_pos[1] << "]");

    // set vel, adjust heading & drive Husky
    setVel(3.0, "forward");
    adjustHeading(ang);
    DriveHusky();

    // viz pillar
    vizPillar();
}


/* @brief: initialize pillar marker in RViz */
void HuskyHighlevelController::initPillarMarker() {
    marker.header.frame_id = "base_laser";
    marker.header.stamp = ros::Time();
    marker.ns = "pillar";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pillar_pos[0];
    marker.pose.position.y = pillar_pos[1];
    marker.pose.position.z = -1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

} /* namespace */
