#ifndef RC_DEVICE_HPP
#define RC_DEVICE_HPP

#include <irl_can_bus/can_robot_device.hpp>
#include <ros/ros.h>

namespace irl_can_ros_ctrl
{
    /// \brief A base class for ros_control-managed CANRobotDevices.
    class RCDevice: public irl_can_bus::CANRobotDevice
    {
    private:
    public:
        RCDevice(const ros::NodeHandle& n);
    };
}

#endif

