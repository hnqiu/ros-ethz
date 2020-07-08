
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


namespace husky_highlevel_controller {

/* @brief: Constructor */
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nh)
    : nodeHandle(nh), subscriber(), publisher(), msg(), ctrl_p(.0) {
    // get param from config file
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
    // create publisher
    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
    publisher.publish(msg);
}


/*@brief: ROS topic callback function
 * print out the position of the pillar with respect to the robot
 */
void HuskyHighlevelController::LaserCallback(const sensor_msgs::LaserScan &msg) {
    float pillar_pos[2];
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
}
} /* namespace */
