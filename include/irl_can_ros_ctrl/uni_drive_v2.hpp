#ifndef UNIDRIVE_V2_HPP
#define UNIDRIVE_V2_HPP

#include <irl_can_bus/can_robot_device.hpp>

namespace irl_can_ros_ctrl
{
    /// \brief A CANRobotDevice for the UniDriveV2 motor driver from IntRoLab.
    class UniDriveV2: public irl_can_bus::CANRobotDevice
    {
    private:
        int  pos_;
        bool pos_ready_;
    public:
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

    public:
        UniDriveV2(int dev_id); 

        void enable(irl_can_bus::CANManager& can);
        void enableCtrl(irl_can_bus::CANManager& can);
        void disableCtrl(irl_can_bus::CANManager& can);
        void disable(irl_can_bus::CANManager& can);
        void requestState(irl_can_bus::CANManager& can);
        bool stateReady();
        void processMsg(const irl_can_bus::LaboriusMessage& msg);

        // TEMP
        void update();
    };
}

#endif

