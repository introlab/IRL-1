#ifndef PICDRIVE_HPP
#define PICDRIVE_HPP

#include "rc_device.hpp"
#include "rc_device_factory.hpp"
#include <irl_can_bus/can_robot_device.hpp>

namespace irl_can_ros_ctrl
{
    /// \brief A CANRobotDevice for the PICDrive motor driver from IntRoLab.
    ///
    /// ROS Parameters (TODO):
    ///  - joint_name: The name of the controlled joint.
    ///                Default: "picdrive_joint"
    ///
    class PICDrive: public RCDevice 
    {
    private:
        // CAN-related defines
        enum
        {
            CAN_CMD_SETPOINT                = 0x84,
            CAN_OUTPUT_DRIVE_STATE1         = 0xFE,
            ADMITTANCE_K_OFFSET             = 104,
            ADMITTANCE_D_OFFSET             = 106,
            ADMITTANCE_I_OFFSET             = 108
        };

        // ROS parameters
        std::string joint_name_;

        /// \brief If a new state has been received.
        bool new_state_;

        /// \brief Conversion factor from the model (rad) to the actuator 
        /// (pulses).
        float pos_conv_to_;
        /// \brief Conversion factor from the actuator (pulses) to the model 
        /// (rad).
        float pos_conv_from_;
        /// \brief Conversion factor from the model (rad/s) to the actuator 
        /// (pulses/s).
        float vel_conv_to_;
        /// \brief Conversion factor from the actuator (pulses/s) to the model 
        /// (rad/s).
        float vel_conv_from_;
        /// \brief Conversion factor from the model (Nm) to the actuator 
        /// (pulses).
        float tqe_conv_to_;
        /// \brief Conversion factor from the actuator (pulses) to the model 
        /// (Nm).
        float tqe_conv_from_;

        /// \brief Set point coming from higher-level controllers.
        double set_point_;

        /// \brief Maximum setpoin in drive units (pulses, torque)
        /// (pulses, ADC units)
        int max_setpoint_;

        /// \brief Minimum setpoint in drive units (pulses, torque)
        /// (pulses, ADC units)
        int min_setpoint_;
        
        /// \brief Reference to the commanded variable. Position by default.
        double* cmd_var_;
        /// \brief Reference to the commanded variable conversion ratio.
        /// Defaults to pos_conv_to_. 
        float* cmd_conv_to_;

        /// \brief The drive's loop time base, used in speed conversions.
        float timebase_;

        /// Internal state.
        double position_;
        double velocity_;
        double torque_;

        unsigned short drive_state_; // Drive operation state.

    public:
        /// \brief Constructor.
        ///
        /// Construct a UniDriveV2 in position control mode (default).
        ///
        /// \param np Namespace for parameters.
        PICDrive(const ros::NodeHandle& np); 

        ~PICDrive();

        static RCDevicePtr create(const ros::NodeHandle& np);

        void registerCtrlIfaces(IRLRobot& robot);
        
        virtual irl_can_bus::ThrottlingDef 
            throttled(const irl_can_bus::TimeBase& p) const;

        /// \brief Return the current (CANRobotDevice) state.
        ///
        /// Depends on the requested state (req_state_) and if the drive is
        /// ready to be used.
        virtual CANRobotDevice::State state() const;

        /// \brief Enable the drive.
        ///
        /// Automatically request conversion ratios.
        /// Switch to enabled state only when all the conversion ratios have
        /// been received.
        void enable(irl_can_bus::CANManager& can);
        
        void enableCtrl(irl_can_bus::CANManager& can);
        void disableCtrl(irl_can_bus::CANManager& can);
        void disable(irl_can_bus::CANManager& can);
        void requestState(irl_can_bus::CANManager& can);
        bool stateReady();
        void processMsg(const irl_can_bus::LaboriusMessage& msg);
        void sendCommand(irl_can_bus::CANManager& can);

    };
}

#endif

