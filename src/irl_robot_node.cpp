#include <irl_can_ros_ctrl/irl_robot.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "irl_robot_node");

    ros::NodeHandle n, np("~");
    irl_can_ros_ctrl::IRLRobot robot(n, np);

    ros::spin();
}

