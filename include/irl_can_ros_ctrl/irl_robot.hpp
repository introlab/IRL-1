#ifndef IRL_ROBOT_HPP
#define IRL_ROBOT_HPP

#include "rc_device.hpp"
#include <irl_can_bus/can_robot.hpp>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>

namespace irl_can_ros_ctrl
{
    /// \brief A ros_control-managed robot using irl_can_bus::CANRobot.
    ///
    /// ROS parameters:
    ///
    ///  - period:  Expected update period, in seconds.
    ///             Default: 0.01 s.
    ///  - ifaces:  An array of string naming the CAN bus interfaces to use.
    ///             Example: ["can0", "can1"]
    ///  - devices: An array of device definitions sub namespaces identifiers. 
    ///             Namespaces will be resolved from the private node handle
    ///             (np in the constructor.)
    ///             For instance, a device named "dev1" will normally refer to
    ///             "/irl_robot/dev1".
    ///             See addDevice() for which parameters are expected in each
    ///             namespace.
    class IRLRobot
    {
    private:
        using CANRobot          = irl_can_bus::CANRobot;
        using CANRobotDevicePtr = irl_can_bus::CANRobotDevice;

        std::unique_ptr<CANRobot> can_robot_;
        std::vector<RCDevicePtr>  devices_;

        // ros_control items
        hardware_interface::JointStateInterface rc_jsi_;

        // General ROS interface
        ros::Timer timer_;
         
    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics and services.
        /// \param np Node handle for parameters.
        IRLRobot(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Create and add a new device based on its ROS namespace
        ///        parameters.
        ///
        /// Parameters expected in the namespace:
        ///  - can_device_type: A driver identifier, string.
        ///                     Example: "UniDriveV2".
        ///  - can_device_id:   CAN bus identifier.
        ///
        /// Each device type expects their own parameters in addition to the
        /// base ones.
        /// See their respective documentation for details.
        void addDevice(const ros::NodeHandle& np);
        
        /// \brief Return a reference to the ros_control JointStateInterface.
        hardware_interface::JointStateInterface& jsi() { return rc_jsi_; }

    private:
        /// \brief Control update loop call, registered as a callback in
        ///        CANRobot.
        ///
        /// Called from the same thread as can_robot_->loopOnce() is called.
        void control();

        /// \brief Timer callback for the main control loop.
        void timerCB(const ros::TimerEvent&);
    };
}

#endif

