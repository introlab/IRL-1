#ifndef UNIDRIVE_V3_HPP
#define UNIDRIVE_V3_HPP

#include "rc_device.hpp"
#include "rc_device_factory.hpp"
#include <irl_can_bus/can_robot_device.hpp>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <irl_can_ros_ctrl/SetAdmittance.h>

namespace irl_can_ros_ctrl
{
    /// \brief Conversion ratio from the the time base value.
    ///        Timer is divided by 256 @ 30 MIPS on UniDriveV3
    static const double UNIDRIVE_V3_TIMEBASE_CONV_FROM = (256.0 / 30000000.0);

    /// \brief A CANRobotDevice for the UniDriveV3 motor driver from IntRoLab.
    ///
    /// ROS Parameters:
    ///  - joint_name:       The name of the controlled joint.
    ///                      Default: "unidrive_v3_joint"
    ///  - command_variable: The variable that is commanded on this drive
    ///                      ("position, "velocity" or "torque")
    ///                      Default: "position".
    ///  - polling:          If the drive is configured in polling mode, where
    ///                      state has to be requested at each cycle.
    ///                      Default: true.
    ///
    class UniDriveV3: public RCDevice 
    {
    private:
        // CAN-related defines
        enum
        {
            MODE_VARIABLE_OFFSET            = 0,
            MODE_VARIABLE_NORMAL            = 1,
            MODE_VARIABLE_IDLE              = 0,
            DRIVE_STATE_OFFSET              = 8,
            SETPOINT_VARIABLE_OFFSET        = 10,
            MAX_SETPOINT_VARIABLE_OFFSET    = 14,
            MIN_SETPOINT_VARIABLE_OFFSET    = 18,
            SPEED_VARIABLE_OFFSET           = 66,
            POSITION_VARIABLE_OFFSET        = 74,
            TORQUE_VARIABLE_OFFSET          = 86,
            ADMITTANCE_M_OFFSET             = 158,
            ADMITTANCE_B_OFFSET             = 162,
            ADMITTANCE_K_OFFSET             = 166,
            TORQUE_OFFSET_OFFSET            = 210,
            POSITION_OFFSET_OFFSET          = 214,
            POSITION_TO_RAD_VARIABLE_OFFSET = 226,
            TORQUE_TO_NM_VARIABLE_OFFSET    = 230,
            TIMEBASE_VARIABLE_OFFSET        = 238
        };

        enum
        {
            RAW_DATA_INDEX,
            TRANS_DATA_INDEX,
            DATA_INDEX_SIZE
        };

        ///TYPE OF COMMAND VARIABLE
        typedef enum
        {
            CMD_VAR_NONE,
            CMD_VAR_VELOCITY,
            CMD_VAR_POSITION,
            CMD_VAR_TORQUE
        } CmdVarType;

        // ROS parameters
        std::string joint_name_;

        /// \brief the CANRobotDevice requested state.
        State req_state_;

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

        /// \brief Maximum setpoin in drive units (pulses, torque)
        /// (pulses, ADC units)
        int max_setpoint_;

        /// \brief Minimum setpoint in drive units (pulses, torque)
        /// (pulses, ADC units)
        int min_setpoint_;

        /// \brief Set point coming from higher-level controllers.
        double set_point_;
        
        CmdVarType cmd_var_type_;

        /// \brief Reference to the commanded variable. Position by default.
        double* cmd_var_;
        /// \brief Reference to the commanded variable conversion ratio.
        /// Defaults to pos_conv_to_. 
        float* cmd_conv_to_;

        /// \brief The drive's loop time base, used in speed conversions.
        float timebase_;

        /// Internal state (raw, transmission).
        double position_[DATA_INDEX_SIZE];
        double velocity_[DATA_INDEX_SIZE];
        double torque_[DATA_INDEX_SIZE];

        /// Transmission state(ratio applied).
        transmission_interface::SimpleTransmission  transmission_pos_;
        transmission_interface::SimpleTransmission  transmission_vel_;
        transmission_interface::SimpleTransmission  transmission_eff_;

        /// Conversion from actuator to joint
        transmission_interface::ActuatorToJointPositionInterface act_to_jnt_pos_;
        transmission_interface::ActuatorToJointVelocityInterface act_to_jnt_vel_;
        transmission_interface::ActuatorToJointEffortInterface act_to_jnt_eff_;

        transmission_interface::ActuatorData actuator_data_;
        transmission_interface::JointData joint_data_;
                       

        float torque_offset_; // Torque offset to send to the drive.
        float pos_offset_; // Position offset, in radians.
        float admittance_m_; // Inertia
        float admittance_b_; // Damping
        float admittance_k_; // Elasticity
        // Indicates if we have to send new admittance values.
        bool admittance_changed_; 
        // Indicates if we have to send a new postion offset value.
        bool pos_offset_changed_;
        unsigned short drive_state_; // Drive operation state.

        unsigned char drive_mode_;

        ros::NodeHandle nh_;
        ros::ServiceServer srv_admittance_;

        enum
        {
            POS_RECEIVED = 1,
            VEL_RECEIVED = 2,
            TQE_RECEIVED = 4,
            STA_RECEIVED = 8,
            MOD_RECEIVED = 16,
            ALL_RECEIVED = POS_RECEIVED | 
                           VEL_RECEIVED | 
                           TQE_RECEIVED | 
                           STA_RECEIVED |
                           MOD_RECEIVED

        };
		/// \brief Indicate if a new state has been obtained from the actuator.
		unsigned int new_state_;

        enum 
        {
            NONE_READY      = 0,
            CONV_POSITION   = 1,
            CONV_TORQUE     = 2,
            TIMEBASE        = 4,
            MOTOR_DRIVE     = 8,
            MAX_SETPOINT    = 16,
            MIN_SETPOINT    = 32,
            POS_OFFSET      = 64,
            CONV_READY      = CONV_POSITION | 
                              CONV_TORQUE   | 
                              TIMEBASE      | 
                              MAX_SETPOINT  | 
                              MIN_SETPOINT  | 
                              POS_OFFSET,
            ALL_READY       = MOTOR_DRIVE   | 
                              CONV_READY
        };
        /// \brief A set of flags indicating if the drive is ready
        unsigned int ready_;

        /// \brief Indicates if the drive is in polling mode or not.
        bool polling_;

    public:
        /// \brief Constructor.
        ///
        /// Construct a UniDriveV3 in position control mode (default).
        ///
        /// \param np Namespace for parameters.
        UniDriveV3(const ros::NodeHandle& np); 

        ~UniDriveV3();

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

    protected:

        void calcConvRatios();

        bool admittanceCB(SetAdmittance::Request& req, SetAdmittance::Response&);

        void setAdmittance(irl_can_bus::CANManager& can);
        
    };
}

#endif

