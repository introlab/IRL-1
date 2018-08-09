#ifndef PTU_DRIVE_HPP
#define PTU_DRIVE_HPP

#include "ptu_motor.hpp"
#include "rc_device.hpp"
#include "rc_device_factory.hpp"
#include <irl_can_bus/laborius_message.hpp>
#include <irl_can_bus/can_robot_device.hpp>

namespace irl_can_ros_ctrl
{
    /// \brief A driver for IRL-1's dual motor PTU controller.
    ///
    /// An instance of this driver creates two PTUMotor CAN devices that it
    /// manages itself.
    /// They are not registered directly with the master CAN manager.
    ///
    /// See PTUMotor for details
    /// 
    /// Parameters:
    /// This device expects two sub namespaces named "pan_motor" and
    /// "tilt_motor".
    /// It also overwrites "can_device_id" in each of those namespaces with the
    /// its own if they are not specified.
    /// See PTUMotor for their content.
    ///
    class PTUDrive: public RCDevice
    {

        std::unique_ptr<PTUMotor> pan_motor_;
        std::unique_ptr<PTUMotor> tilt_motor_;
    public:
        PTUDrive(const ros::NodeHandle& n);

        virtual void registerCtrlIfaces(IRLRobot& robot);

        virtual void requestState(irl_can_bus::CANManager& can);
        virtual bool stateReady(); 
        virtual void sendCommand(irl_can_bus::CANManager& can);
        virtual void enable(irl_can_bus::CANManager& can);
        virtual void disable(irl_can_bus::CANManager& can);
        virtual void enableCtrl(irl_can_bus::CANManager& can);
        virtual void disableCtrl(irl_can_bus::CANManager& can);
        virtual void processMsg(const irl_can_bus::LaboriusMessage& msg);

        static RCDevicePtr create(const ros::NodeHandle& np);

    };

}

#endif
