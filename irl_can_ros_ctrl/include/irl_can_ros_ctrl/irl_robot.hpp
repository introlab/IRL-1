#ifndef IRL_ROBOT_HPP
#define IRL_ROBOT_HPP

#include "rc_device.hpp"
#include <irl_can_bus/can_robot.hpp>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
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
    class IRLRobot: public hardware_interface::RobotHW
    {
    private:
        using CANRobot          = irl_can_bus::CANRobot;
        using CANRobotDevicePtr = irl_can_bus::CANRobotDevice;

        std::unique_ptr<CANRobot> can_robot_;
        std::vector<RCDevicePtr>  devices_;

        // ros_control items
        using HWI    = hardware_interface::HardwareInterface;
        using HWIPtr = std::shared_ptr<HWI>; 

        std::vector<HWIPtr>                       rc_hwis_;
        hardware_interface::JointStateInterface   rc_jsi_;
        controller_manager::ControllerManager     rc_cm_;

        // General ROS interface
        ros::Duration period_;

        // Real-time thread
        std::thread rt_thread_;

        bool running_;
         
    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics and services.
        /// \param np Node handle for parameters.
        IRLRobot(ros::NodeHandle& n, ros::NodeHandle& np);


        ~IRLRobot();

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
        
        virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const;        

        void stop();

        /// \brief Return, or create and register, a pointer to an instance of a
        ///        specific ros_control interface.
        ///
        /// Interfaces are created on demand as we cannot determine which will
        //// be necessary at build time (except for the JointStateInterface).
        template <class T>
        T* getHWI()
        {
            T* hwi = get<T>();
            if (!hwi) {
                // Create and register.
                hwi = new T();
                HWIPtr hwi_p(hwi);
                rc_hwis_.push_back(hwi_p);
                registerInterface(hwi);
            }

            return hwi;
        }

        /// \brief Return a reference to the ros_control JointStateInterface.
        ///
        /// For other types of HardwareInterface, see getHWI().
        hardware_interface::JointStateInterface& jsi() { return rc_jsi_; }

    private:
        /// \brief Control update loop call, registered as a callback in
        ///        CANRobot.
        ///
        /// Called from the same thread as can_robot_->loopOnce() is called.
        void control();

        /// \brief Real-time thread loop.
        void rtThread();
    };
}

#endif

