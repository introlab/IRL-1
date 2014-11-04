#include <ros/ros.h>
#include <irl_can_bus/can_robot.hpp>
#include <signal.h>

namespace
{
    irl_can_bus::CANRobot* robot_ = 0;

    void signalHandler(int)
    {
        if (robot_) {
            robot_->stop();
        }

        ros::shutdown();
    }
    
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
    ros::init(argc, 
              argv,
              "can_robot_proto",
              ros::init_options::NoSigintHandler);
    signal(SIGINT, signalHandler);

    irl_can_bus::log::loggerFunction(&::loggerFunction);

    ros::NodeHandle n, np("~");

    std::vector<std::string> ifaces;
    if (np.hasParam("ifaces")) {
        XmlRpc::XmlRpcValue ifnames_arr; 
        np.getParam("ifaces", ifnames_arr);
        if (ifnames_arr.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < ifnames_arr.size(); ++i) {
                XmlRpc::XmlRpcValue& ifname = ifnames_arr[i];
                if (ifname.getType() == XmlRpc::XmlRpcValue::TypeString) {
                    ifaces.push_back(std::string(ifname));
                } else {
                    ROS_WARN("~ifaces element is not a string.");
                }
            }
        } else {
            ROS_WARN("~ifaces is not an array.");
        }
    }

    if (ifaces.empty()) {
        ROS_WARN("Empty interfaces list (~ifaces parameter).");
    }

    robot_ = new irl_can_bus::CANRobot(ifaces);

    while (ros::ok()) {
        robot_->loopOnce();
        ros::spinOnce();
    }

    delete robot_;
}

