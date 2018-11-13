#ifndef FACE_CTRL_RC_HPP
#define FACE_CTRL_RC_HPP

#include "face_ctrl_base.hpp"
#include <irl_can_ros_ctrl/rc_device.hpp>
#include <irl_can_ros_ctrl/rc_device_factory.hpp>
#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <irl_can_bus/can_robot_device.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace jn0_face_ctrl
{
    using RCDevice      = irl_can_ros_ctrl::RCDevice;
    using RCDevicePtr   = irl_can_ros_ctrl::RCDevicePtr;

    /// \brief A ros_control CANRobotDevice driver for the JN0 face.
    ///
    /// For parameter details, see the base FaceCtrl class.
    ///
    /// While this is a ros_control device, it does not expose or tie to 
    /// ros_control interfaces.
    ///
    /// As the face servo controller is a very simple device, this driver can
    /// most of the basic interface of the RCDevice (and CANRobotDevice) class.
    ///
    /// RC-specific parameters:
    /// 
    ///  - clock_divider: Only send pose update every N control cycles.
    ///                   Default: 10.
    ///
    /// Topic:
    ///  - cmd_face:      New face pose to apply.
    ///  - eyes_target:   New 3D point to look at (always overwrites the rest
    ///                   of the pose).
    class FaceCtrlRC: public RCDevice, public FaceCtrlBase
    {
    private:
        int     clock_divider_;
        int     cycle_i_;
        bool    new_pose_;

        realtime_tools::RealtimeBuffer<jn0_face_msgs::FacePose> pose_buffer_;
        realtime_tools::RealtimeBuffer<geometry_msgs::Point>    target_buffer_;
        
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_target_;

    public:
        /// \brief Default constructor.
        ///
        /// Does not result in a working driver.
        /// Note that pluginlib needs a defined default constructor to work.
        FaceCtrlRC() {}

        /// \brief Constructor.
        ///
        /// \param np Namespace for parameters.
        FaceCtrlRC(const ros::NodeHandle& np); 

        ~FaceCtrlRC() {};

        /// \brief RCDevice factory helper function.
        static RCDevicePtr create(const ros::NodeHandle& np);

        /// \brief Interface registration callback (does nothing).
        void registerCtrlIfaces(irl_can_ros_ctrl::IRLRobot& robot);
        
        bool stateReady() { return true; }

        void processMsg(const irl_can_bus::LaboriusMessage& msg);
        void sendCommand(irl_can_bus::CANManager& can);

    private:
        void poseCB(const jn0_face_msgs::FacePose& msg);
        void targetCB(const geometry_msgs::Point& msg);

    };
}

#endif

