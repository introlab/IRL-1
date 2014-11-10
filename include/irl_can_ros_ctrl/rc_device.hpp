#ifndef RC_DEVICE_HPP
#define RC_DEVICE_HPP

#include <irl_can_bus/can_robot_device.hpp>
#include <ros/ros.h>

namespace irl_can_ros_ctrl
{
    class IRLRobot;

    /// \brief A base class for ros_control-managed CANRobotDevices.
    class RCDevice: public irl_can_bus::CANRobotDevice
    {
    private:
    public:
        RCDevice(const ros::NodeHandle& n);

        ~RCDevice();

        /// \brief Called by the IRLRobot instance to register ros_control
        /// interfaces.
        virtual void registerCtrlIfaces(IRLRobot& robot) = 0;
    };

    using RCDevicePtr = std::shared_ptr<RCDevice>;
}

#endif

