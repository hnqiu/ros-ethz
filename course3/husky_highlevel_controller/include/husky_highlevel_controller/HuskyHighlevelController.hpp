#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
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

private:
    /* data */
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    geometry_msgs::Twist msg;
    float ctrl_p;

    /* @brief: ROS topic callback function 
     * print out the position of the pillar with respect to the robot
     */
    void LaserCallback(const sensor_msgs::LaserScan &msg);
};

} /* namespace */
