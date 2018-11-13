#ifndef PTU_MOTOR_HPP
#define PTU_MOTOR_HPP

#include "rc_device.hpp"
#include "rc_device_factory.hpp"
#include <irl_can_bus/laborius_message.hpp>
#include <irl_can_bus/can_robot_device.hpp>

namespace irl_can_ros_ctrl
{
    /// \brief A driver for each of the motors of IRL-1's PTU controller.
    ///
    /// Can only be used with PTUDrive, not as a standalone instance.
    /// Each instance manages a subset of variables on the same CAN
    /// device.
    /// The base offset parameter specifies which motor to control.
    /// State and synchronisation is managed by the PTUDrive.
    ///
    /// States:
    /// 
    ///  - STATE_DISABLED: At init.
    ///    Switches to STATE_STARTING with a start() call.
    ///  - STATE_STARTING: Wait for a current position value, does not apply
    ///    set points.
    ///    When a valid position has been received, send a set point to the
    ///    latest value, enables the motor and switch to STATE_CONTROL.
    ///  - STATE_CONTROL: Applies set points.
    ///    Switches to STATE_CONTROL after a stop() call.
    /// 
    /// Parameters:
    ///  - motor_enabled: Indicates if the driver should send set point commands
    ///    or not.
    ///    Default: true.
    ///  - base_offset: Select which drive to control.
    ///    The offset is based at the "Model Number" variable.
    ///    Default: 0, which selects the pan actuator. 
    ///    To select the tilt actuator, the offset is 66.
    ///  - position_conv_ratio: The position conversion ratio to the drive,
    ///    in ticks/rad.
    ///    Default: 195.3788, or 1023 / (300 / 180 * pi) for the pan actuator.
    ///    The tilt actuator has a different range, the ratio is 935.0646, or 
    ///    4095 / (250.92 / 180 * pi).
    ///  - position_center: The position offset for 0 rad in ticks.
    ///    Default: 512, which is the center of the pan actuator.
    ///    The tilt center is at 2048.
    ///  - velocity_conv_ratio: The velocity conversion ratio to use for values
    ///    incoming from the drive.
    ///    Default: 0.0116, or (0.111 / 60 * 2.0 * pi).
    ///    The 0.111 coefficient converts the incoming value into RPM.
    ///  - max_velocity: Maximum velocity allowed in rad/s.
    ///    This limit is managed by the actuator itself, the parameter value is
    ///    simply sent to it in RPM.
    ///    NOTE: A max velocity of 0 sets the drive to maximum velocity.
    ///    Default: 1.047 rad/s, or roughly 10 RPM.
    ///  - rest_angle: Angle to set before shutdown, in radians.
    ///    Default 0.0 rad.
    ///  - stop_at_shutdown: Stop the motor at this driver's shutdown.
    ///    Default: true.
    ///  - clock_divider: Ratio that divides update calls from CANMotorManager 
    ///    to actual calls for this device.
    ///    Should match an actual refresh rate of ~10 Hz.
    ///    For instance, if the CANMotorManager's control period is set to 10000
    ///    us (the default value), this ratio should be set to 10.
    ///    Default: 10. 
    ///
    class PTUMotor: public RCDevice
    {
        enum
        {
            TORQUE_ENABLE_OFFSET    = 27,
            SET_POINT_OFFSET        = 28,
            MOVING_SPEED_OFFSET     = 32,
            CUR_POS_OFFSET          = 40,
            CUR_VEL_OFFSET          = 44
        };

        std::string joint_name_;

        double cur_pos_;
        double cur_vel_;
        double cur_eff_; // Always 0 torque for now.
        double cur_cmd_;

        bool motor_enabled_;
        unsigned char base_offset_;
        double pos_conv_to_;
        double pos_conv_from_;
        int pos_center_;
        double vel_conv_to_;
        double vel_conv_from_;
        double max_vel_;
        double rest_angle_;
        bool stop_at_shutdown_;

        bool state_ready_;

        int clock_divider_;
        int cycle_;

        int device_id_;

    public:
        /// \brief Constructor.
        PTUMotor(const ros::NodeHandle& n);

        virtual void registerCtrlIfaces(IRLRobot& robot);

        virtual void requestState(irl_can_bus::CANManager& can);
        virtual bool stateReady() { return state_ready_; }
        virtual void sendCommand(irl_can_bus::CANManager& can);
        virtual void enable(irl_can_bus::CANManager& can);
        virtual void disable(irl_can_bus::CANManager& can);
        virtual void enableCtrl(irl_can_bus::CANManager& can);
        virtual void disableCtrl(irl_can_bus::CANManager& can);
        virtual void processMsg(const irl_can_bus::LaboriusMessage& msg);

        static RCDevicePtr create(const ros::NodeHandle& np);

    private:
        void startMotor(irl_can_bus::CANManager& can);
        void stopMotor(irl_can_bus::CANManager& can);
        void setPoint(irl_can_bus::CANManager& can, double val);
        void setMaxVel(irl_can_bus::CANManager& can);


    };

}

#endif
