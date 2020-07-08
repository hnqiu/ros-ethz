#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);
    ros::Rate rate(2);
    huskyHighlevelController.setVel(1.0, "forward");
    huskyHighlevelController.setVel(1.0, "ang");

    while (ros::ok()) {
        huskyHighlevelController.DriveHusky();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
