#include <irl_can_ros_ctrl/uni_drive_v3.hpp>
#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <irl_can_bus/can_base_macros.h>
#include <ros/ros.h>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

namespace {
    static RCDeviceCreatorHelper ch_("UniDriveV3", 
                                     RCDeviceCreator(&UniDriveV3::create));
}

UniDriveV3::UniDriveV3(const ros::NodeHandle& np): 
    RCDevice(np),
    cmd_var_(&position_),
    cmd_conv_to_(&pos_conv_to_),
    polling_(true)
{
    np.param("joint_name", joint_name_, std::string("unidrive_v3_joint"));

    std::string cmd_var;
    np.param("command_variable", cmd_var, std::string("position"));

    // TODO: Get/Set this from/to the drive.
    if (cmd_var == "position")
    {
        cmd_var_ = &position_;
        cmd_conv_to_ = &pos_conv_to_;
    }
    else if (cmd_var == "velocity")
    {
        cmd_var_ = &velocity_;
        cmd_conv_to_ = &vel_conv_to_;
    }
    else if (cmd_var == "torque")
    {
        cmd_var_ = &torque_;
        cmd_conv_to_ = &tqe_conv_to_;
    }

    np.param("polling", polling_, true);

    ROS_INFO("Joint name : %s, type: %s, polling: %i",joint_name_.c_str(), cmd_var.c_str(), polling_);
}

UniDriveV3::~UniDriveV3()
{
}

RCDevicePtr UniDriveV3::create(const ros::NodeHandle& np)
{
    ROS_INFO("UniDriveV3::create");
    return RCDevicePtr(new UniDriveV3(np));
}

void UniDriveV3::registerCtrlIfaces(IRLRobot& robot)
{

    //JOINT STATE INTERFACE
    hardware_interface::JointStateHandle sh(joint_name_, 
                                            &position_,
                                            &velocity_, 
                                            &torque_);
    robot.jsi().registerHandle(sh);


    //JOINT COMMAND INTERFACE    
    hardware_interface::JointCommandInterface* jci = nullptr;

    if (cmd_var_ == &position_) {
        using HWI = hardware_interface::PositionJointInterface;
        jci = robot.getHWI<HWI>();
    } else if (cmd_var_ == &velocity_) {
        using HWI = hardware_interface::VelocityJointInterface;
        jci = robot.getHWI<HWI>();
    } else if (cmd_var_ == &torque_) {
        using HWI = hardware_interface::EffortJointInterface;
        jci = robot.getHWI<HWI>();
    }

    if (jci != nullptr) {
        jci->registerHandle(hardware_interface::JointHandle(sh,
                                                            &set_point_));
    }

}

ThrottlingDef UniDriveV3::throttled(const TimeBase& p) const
{
    // UniDriveV3 limited to 1 message every 1 ms, meaning the maximum
    // state update frequency is 250 Hz (4 messages: 4 ms).
    ThrottlingDef td = {TimeBase(1000), 1};
    return td;
}

CANRobotDevice::State UniDriveV3::state() const
{
    return CANRobotDevice::state();
}

void UniDriveV3::enable(CANManager& can)
{
    ROS_INFO("enable %i",deviceID());
    req_state_ = STATE_ENABLED;
    CANRobotDevice::state(STATE_STARTING);

    position_      = 0.0;
    velocity_      = 0.0;
    torque_        = 0.0;
    torque_offset_ = 0.0;
    pos_offset_    = 0.0;

    //SETPOINT = CURRENT VALUE
    set_point_     = *cmd_var_;

    admittance_changed_ = true;
    pos_offset_changed_ = false;

    ready_ = NONE_READY; 

    can.requestMem(deviceID(), POSITION_TO_RAD_VARIABLE_OFFSET, sizeof(float));
    can.requestMem(deviceID(), TORQUE_TO_NM_VARIABLE_OFFSET,    sizeof(float));
    can.requestMem(deviceID(), TIMEBASE_VARIABLE_OFFSET,        sizeof(short));
    can.requestMem(deviceID(), MAX_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), MIN_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), POSITION_OFFSET_OFFSET,          sizeof(int));
}

void UniDriveV3::enableCtrl(CANManager& can)
{
    req_state_ = STATE_CONTROL;
    CANRobotDevice::enableCtrl(can);

    // TODO: Start motor.
    ROS_INFO("enableCtrl");

    // Start the motor:
/*
    LaboriusMessage msg;
    msg.msg_priority    = 0;
    msg.msg_type        = CAN_TYPE_ACTUATOR_HIGH_PRIORITY;
    msg.msg_cmd         = 0x80;
    msg.msg_boot        = 0;
    msg.msg_dest        = deviceID();
    msg.msg_data_length = 1;
    msg.msg_data[0]     = 1;
    msg.msg_remote      = 0;
    can.pushOneMessage(msg);
*/

}

void UniDriveV3::disableCtrl(CANManager& can)
{
    req_state_ = STATE_ENABLED;
    CANRobotDevice::disableCtrl(can);

    // TODO: Stop motor.
    ROS_INFO("disableCtrl");

/*
    LaboriusMessage msg;
    msg.msg_priority    = 0;
    msg.msg_type        = CAN_TYPE_ACTUATOR_HIGH_PRIORITY;
    msg.msg_cmd         = 0x80;
    msg.msg_boot        = 0;
    msg.msg_dest        = deviceID();
    msg.msg_data_length = 1;
    msg.msg_data[0]     = 1;
    msg.msg_remote      = 0;
    can.pushOneMessage(msg);
*/

}

void UniDriveV3::disable(CANManager& can)
{
    req_state_ = STATE_DISABLED;
    CANRobotDevice::disable(can);
}

void UniDriveV3::requestState(CANManager& can)
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

    //ROS_INFO("requestState : id: %i",deviceID());

}

bool UniDriveV3::stateReady()
{

    //ROS_INFO("STATE_READY : %i",new_state_ == ALL_RECEIVED);

    if (new_state_ == ALL_RECEIVED) {
        return true;
    } else {
        return false;
    }
}

void UniDriveV3::processMsg(const LaboriusMessage& msg)
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
            if (ready_ == CONV_READY) 
            {
                velocity_ = vel_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= VEL_RECEIVED;
            }
        break;
        
        case POSITION_VARIABLE_OFFSET:
            if (ready_ == CONV_READY)
            {
                position_ = pos_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= POS_RECEIVED;
            }
        break;

        case TORQUE_VARIABLE_OFFSET:
            if (ready_ == CONV_READY)
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
            timebase_ = UNIDRIVE_V3_TIMEBASE_CONV_FROM *
                *((unsigned short*)msg.msg_data);
            ready_ |= TIMEBASE;
        break;
        
        case DRIVE_STATE_OFFSET:
            drive_state_ = *((unsigned short*)msg.msg_data);  
            new_state_ |= STA_RECEIVED;

            if (drive_state_ != 0)
                ROS_ERROR_THROTTLE(1.0,
                                   "UniDriveV3 %i state: %i", 
                                   deviceID(), 
                                   drive_state_);
        break;

        default:
            ROS_WARN_THROTTLE(1.0,
                              "Received an unknown variable on UniDriveV3 %i, "
                              "offset: %i", 
                              deviceID(),
                              msg.msg_cmd);
        break;
    };

    //ROS_WARN("Received, new_state: %i", new_state_);
    if (state() == STATE_STARTING) {
        ROS_WARN("Received, ready: %i (conv: %i).", ready_, CONV_READY);
        if (ready_ == CONV_READY) {
            ROS_DEBUG("Switching UniDriveV3 %i to ENABLED.", deviceID());
            ROS_WARN("Switching to ENABLED");
            CANRobotDevice::state(STATE_ENABLED);
        }
    }

}

void UniDriveV3::sendCommand(CANManager& can)
{
    ROS_INFO_THROTTLE(5.0,"SendCommand : %f",set_point_);
}

