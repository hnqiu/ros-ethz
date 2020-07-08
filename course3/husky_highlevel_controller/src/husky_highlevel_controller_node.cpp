#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
