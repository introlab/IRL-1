#ifndef UNIDRIVE_V2_HPP
#define UNIDRIVE_V2_HPP

#include <irl_can_bus/can_robot_device.hpp>

namespace irl_can_ros_ctrl
{
    /// \brief Conversion ratio from the the time base value.
    static const double UNIDRIVE_V2_TIMEBASE_CONV_FROM = 1.28e-5;

    /// \brief A CANRobotDevice for the UniDriveV2 motor driver from IntRoLab.
    class UniDriveV2: public irl_can_bus::CANRobotDevice
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
        
        /// \brief Reference to the commanded variable. Position by default.
        float* cmd_var_;
        /// \brief Reference to the commanded variable conversion ratio.
        /// Defaults to pos_conv_to_. 
        float* cmd_conv_to_;

        /// \brief The drive's loop time base, used in speed conversions.
        float timebase_;

        /// Internal state.
        float position_;
        float velocity_;
        float torque_;
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

        enum
        {
            POS_RECEIVED = 1,
            VEL_RECEIVED = 2,
            TQE_RECEIVED = 4,
            STA_RECEIVED = 8,
            ALL_RECEIVED = POS_RECEIVED | 
                           VEL_RECEIVED | 
                           TQE_RECEIVED | 
                           STA_RECEIVED
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

        // \brief Set to true to consider torque offset changes
        bool active_torque_offset_; 
        // \brief Indicates if the torque offset values should be converted.
        bool convert_torque_offset_;
        /// \brief m in torque_offset_out = m * torque_offset_in + b
        double torque_offset_km_;
        /// \brief b in torque_offset_out = m * torque_offset_in + b
        double torque_offset_kb_;
        /// \brief Indicates if the drive is in polling mode or not.
        bool polling_;

    public:
        /// \brief Constructor.
        ///
        /// Construct a UniDriveV2 in position control mode (default).
        ///
        /// \param dev_id The CAN device id.
        UniDriveV2(int dev_id); 

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

        // TEMP
        double pos() const { return position_; }
    };
}

#endif

