#ifndef IRL_ROBOT_HPP
#define IRL_ROBOT_HPP

#include <irl_can_bus/can_robot.hpp>
#include <ros/ros.h>

namespace irl_can_ros_ctrl
{
    /// \brief A ros_control-managed robot using irl_can_bus::CANRobot.
    class IRLRobot
    {
    private:
        using CANRobot          = irl_can_bus::CANRobot;
        using CANRobotDevicePtr = irl_can_bus::CANRobotDevice;

        CANRobot                       can_robot_;
        std::vector<CANRobotDevicePtr> devices_;

    public:
        IRLRobot();

        /// \brief Create and add a new device based on its ROS namespace
        ///        parameters.
        ///
        /// Parameters expected in the namespace:
        ///  - can_device_type: A driver identifier, string.
        ///                     Example: "UniDriveV2".
        ///  - can_device_id:   CAN bus identifier.
        void addDevice(ros::NodeHandle& np);
    };
}

#endif

