//
// Created by larr-lyw on 21. 8. 11..
//

#include "ros/ros.h"
#include "pd_position_controller_simple_topic.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_position_controller_simple_topic_node");
    PDController controller;
    controller.run();
}
