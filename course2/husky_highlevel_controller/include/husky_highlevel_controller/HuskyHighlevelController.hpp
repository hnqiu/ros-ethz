/* Copyright (C) 2019-2020 hnqiu. All Rights Reserved.
 * Licensed under the BSD-3-Clause license. See LICENSE for details.
 */


#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace husky_highlevel_controller {

/* @brief: Husky Highlevel Controller */
class HuskyHighlevelController {
public:
    /* @brief: Constructor */
    HuskyHighlevelController(ros::NodeHandle& nh);

    /* @brief: Copy constructor
     * defined as DELETED for simplicity 
     * @comments: at this moment, I don't think this is necessarily a good
     * practice to wrap up node handlers, subscribers etc. into a new class,
     * because we should be very careful with the behavior of copy constructors.
     * However, if we really want to copy this class, remember to
     * copy both vars 'nodeHandle' & 'subscriber'. Do NOT initialize
     * the subscriber again from the method 'NodeHandle.subscribe()'.
     * Refer to the source code
     * http://wiki.ros.org/roscpp/Overview/NodeHandles
     * https://github.com/ros/ros_comm/blob/kinetic-devel/clients/roscpp/src/libros/node_handle.cpp
     * https://github.com/ros/ros_comm/blob/kinetic-devel/clients/roscpp/src/libros/subscriber.cpp
     */
    HuskyHighlevelController(const HuskyHighlevelController &) = delete;
    /* @brief: no assignment = */
    HuskyHighlevelController& operator=(const HuskyHighlevelController &) = delete;

    /* @brief: Destructor */
    ~HuskyHighlevelController() = default;

private:
    /* data */
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;

    /* @brief: ROS topic callback function 
     * print out the distance to the closest object
     */
    void LaserCallback(const sensor_msgs::LaserScan &msg);
};

} /* namespace */
