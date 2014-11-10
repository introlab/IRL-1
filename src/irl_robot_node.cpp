#include <irl_can_ros_ctrl/irl_robot.hpp>

namespace {
    void loggerFunction(irl_can_bus::log::LogID id, const char* str)
    {
        switch (id) {
            case irl_can_bus::log::CAN_LOG_DEBUG:
                ROS_DEBUG_STREAM(str);
                break;
            case irl_can_bus::log::CAN_LOG_WARN:
                ROS_WARN_STREAM(str);
                break;
            case irl_can_bus::log::CAN_LOG_ERROR:
                ROS_ERROR_STREAM(str);
                break;
            default:
                ROS_INFO_STREAM(str);
                break;
        };
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "irl_robot_node");
    irl_can_bus::log::loggerFunction(&::loggerFunction);

    ros::NodeHandle n, np("~");
    irl_can_ros_ctrl::IRLRobot robot(n, np);

    ros::spin();
}

