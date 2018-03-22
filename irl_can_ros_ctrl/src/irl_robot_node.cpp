#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <signal.h>

irl_can_ros_ctrl::IRLRobot *g_robot =  NULL;

namespace {
    void loggerFunction(irl_can_bus::log::LogID id, const char* str)
    {
        switch (id) {
            case irl_can_bus::log::CAN_LOG_DEBUG:
                ROS_INFO_STREAM(str);
                break;
            case irl_can_bus::log::CAN_LOG_INFO:
                ROS_INFO_STREAM(str);
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

    // Replacement SIGINT handler
    void mySigIntHandler(int sig)
    {
        ROS_INFO("SIGINT caught");

        if (g_robot) {
            g_robot->stop();
        }

        ros::shutdown();  
    }

}
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "irl_robot_node", ros::init_options::NoSigintHandler);
    //ros::init(argc, argv, "irl_robot_node");
    signal(SIGINT, mySigIntHandler);

    irl_can_bus::log::loggerFunction(&::loggerFunction);

    ros::NodeHandle n, np("~");
    irl_can_ros_ctrl::IRLRobot robot(n, np);

    g_robot = &robot;

    ros::spin();

    ROS_WARN_STREAM("spin done");
    cerr<<"spin done"<<endl;

    return 0;
}

