#include <ros/ros.h>
#include <irl_can_bus/can_robot.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_robot_proto");

    ros::NodeHandle n, np("~");

    ros::spin();
}
