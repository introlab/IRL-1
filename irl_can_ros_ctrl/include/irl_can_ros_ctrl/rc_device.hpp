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
    public:
        RCDevice(const ros::NodeHandle& n);

        ~RCDevice();

        /// \brief Called by the IRLRobot instance to register ros_control
        /// interfaces.
        ///
        /// Each driver has to register its joint handles on the robot's state
        /// interface, and command interfaces either on the generic (JCI) one
        /// provided by the robot or custom ones directly through the RobotHW
        /// base class interface of IRLRobot.
        virtual void registerCtrlIfaces(IRLRobot& robot) = 0;
    };

    using RCDevicePtr = std::shared_ptr<RCDevice>;
}

#endif

