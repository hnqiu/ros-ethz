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
