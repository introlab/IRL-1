#include <irl_can_bus/can_robot.hpp>
#include <irl_can_ros_ctrl/uni_drive_v2.hpp>
#include <ros/ros.h>
#include <signal.h>

namespace
{
    irl_can_bus::CANRobot*         robot_ = 0;
    irl_can_bus::CANRobotDevicePtr drives_[4];

    void signalHandler(int)
    {
        if (robot_) {
            robot_->stop();
        }

        ros::shutdown();
    }
    
    void loggerFunction(irl_can_bus::log::LogID id, const char* str)
    {
        ROS_INFO_STREAM(str);

        return;

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

    void ctrlCB()
    {
        ROS_INFO_THROTTLE(1.0, "Running...");
        using namespace irl_can_ros_ctrl;
        for (int i = 0; i < 4; ++i) {
            if (drives_[i]) {

                UniDriveV2* d = static_cast<UniDriveV2*>(drives_[i].get());

                /*
                if (d->state() == irl_can_bus::CANRobotDevice::STATE_ENABLED) {
                    ROS_INFO("Drive %i pos: %f",
                             d->deviceID(), 
                             d->pos());
                }
                */
            }
        }
    }

}

int main(int argc, char** argv)
{
    using namespace irl_can_ros_ctrl;

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

    for (int i = 0; i < 4; ++i) {
        std::stringstream np_name;
        np_name << "dev_" << i;
        ros::NodeHandle nd(np, np_name.str());
        drives_[i].reset(new UniDriveV2(nd));
        robot_->addDevice(drives_[i]);
    }

    robot_->registerCtrlCB(&::ctrlCB);

    robot_->start();

    while (ros::ok()) {
        robot_->loopOnce();
        ROS_DEBUG_THROTTLE(1.0, "Still looping...");
        ros::spinOnce();
        ros::Rate(100).sleep();
    }

    delete robot_;
}

