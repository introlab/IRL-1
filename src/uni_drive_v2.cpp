#include <irl_can_ros_ctrl/uni_drive_v2.hpp>
#include <irl_can_bus/can_base_macros.h>
#include <ros/ros.h>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

UniDriveV2::UniDriveV2(int dev_id): 
    CANRobotDevice(dev_id),
    cmd_var_(&position_),
    cmd_conv_to_(&pos_conv_to_)
{
}

CANRobotDevice::State UniDriveV2::state() const
{
    if (req_state_ == STATE_ENABLED) {
        return ready_ == ALL_READY ? STATE_ENABLED
                                   : STATE_DISABLED;
    } else {
        return CANRobotDevice::state();
    }

}

void UniDriveV2::enable(CANManager& can)
{
    req_state_ = STATE_ENABLED;

    position_      = 0.0;
    velocity_      = 0.0;
    torque_        = 0.0;
    torque_offset_ = 0.0;
    pos_offset_    = 0.0;

    admittance_changed_ = true;
    pos_offset_changed_ = false;

    ready_     = NONE_READY; 

    can.requestMem(deviceID(), POSITION_TO_RAD_VARIABLE_OFFSET, sizeof(float));
    can.requestMem(deviceID(), TORQUE_TO_NM_VARIABLE_OFFSET,    sizeof(float));
    can.requestMem(deviceID(), TIMEBASE_VARIABLE_OFFSET,        sizeof(short));
    can.requestMem(deviceID(), MAX_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), MIN_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), POSITION_OFFSET_OFFSET,          sizeof(int));
}

void UniDriveV2::enableCtrl(CANManager& can)
{
    req_state_ = STATE_CONTROL;
    CANRobotDevice::enableCtrl(can);

    // TODO: Start motor.
}

void UniDriveV2::disableCtrl(CANManager& can)
{
    req_state_ = STATE_ENABLED;
    CANRobotDevice::disableCtrl(can);

    // TODO: Stop motor.
}

void UniDriveV2::disable(CANManager& can)
{
    req_state_ = STATE_DISABLED;
    CANRobotDevice::disable(can);
}

void UniDriveV2::requestState(CANManager& can)
{
    new_state_ = 0;

    // In non-polling mode, messages should come by themselves.
    // Do not send an extra request.
    if (!polling_)
        return;
    can.requestMem(deviceID(), POSITION_VARIABLE_OFFSET, sizeof(int));
    can.requestMem(deviceID(), SPEED_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), TORQUE_VARIABLE_OFFSET,   sizeof(int));
    can.requestMem(deviceID(), DRIVE_STATE_OFFSET,       sizeof(short));

    //last_request_ = ros::Time::now();
}

bool UniDriveV2::stateReady()
{
    return new_state_ == ALL_RECEIVED;
}

void UniDriveV2::processMsg(const LaboriusMessage& msg)
{
    if (msg.msg_type != CAN_TYPE_REQUEST_DATA) {
        return;
    }
    if ((msg.msg_boot & CAN_REQUEST_READ) == 0) {
        return;
    }
    if (msg.msg_data_length <= 0) {
        ROS_ERROR("Received a data request response of length 0 or "
                  "less, skipping.");
        return;
    }

    switch(msg.msg_cmd)
    {
        case MODE_VARIABLE_OFFSET:
            // TODO: Check, and if necessary, switch command variable.
        break;

        case SETPOINT_VARIABLE_OFFSET:
            // TODO: Save this somewhere
        break;

        case MAX_SETPOINT_VARIABLE_OFFSET:
            ready_ |= MAX_SETPOINT;
            max_setpoint_ = *((int*)msg.msg_data);
        break;

        case MIN_SETPOINT_VARIABLE_OFFSET:
            ready_ |= MIN_SETPOINT;
            min_setpoint_ = *((int*)msg.msg_data);
        break;


        case SPEED_VARIABLE_OFFSET:
            if (ready_ == ALL_READY) 
            {
                velocity_ = vel_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= VEL_RECEIVED;
            }
        break;
        
        case POSITION_VARIABLE_OFFSET:
            if (ready_ == ALL_READY)
            {
                position_ = pos_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= POS_RECEIVED;
            }
        break;

        case TORQUE_VARIABLE_OFFSET:
            if (ready_ == ALL_READY)
            {
                torque_ = tqe_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= TQE_RECEIVED;
            }
        break;

        case POSITION_OFFSET_OFFSET:
            pos_offset_ = pos_conv_from_ * *((int*)msg.msg_data);
            ready_ |= POS_OFFSET;
        break;

        case POSITION_TO_RAD_VARIABLE_OFFSET:
            pos_conv_from_ = *((float*)msg.msg_data);
            ready_ |= CONV_POSITION;
        break;

        case TORQUE_TO_NM_VARIABLE_OFFSET:
            tqe_conv_from_ = *((float*)msg.msg_data);
            ready_ |= CONV_TORQUE;
        break;

        case TIMEBASE_VARIABLE_OFFSET:
            timebase_ = UNIDRIVE_V2_TIMEBASE_CONV_FROM *
                *((unsigned short*)msg.msg_data);
            ready_ |= TIMEBASE;
        break;
        
        case DRIVE_STATE_OFFSET:
            drive_state_ = *((unsigned short*)msg.msg_data);  
            if (drive_state_ != 0)
                ROS_ERROR_THROTTLE(1.0,
                                   "UniDriveV2 %i state: %i", 
                                   deviceID(), 
                                   drive_state_);
        break;

        default:
            ROS_WARN_THROTTLE(1.0,
                              "Received an unknown variable on UniDriveV2 %i, "
                              "offset: %i", 
                              deviceID(),
                              msg.msg_cmd);
        break;
    };
}

