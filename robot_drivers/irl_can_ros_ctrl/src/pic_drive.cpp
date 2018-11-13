#include <irl_can_ros_ctrl/pic_drive.hpp>
#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <irl_can_bus/can_base_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

namespace {
    static RCDeviceCreatorHelper ch_("PICDrive", 
                                     RCDeviceCreator(&PICDrive::create));
}

PICDrive::PICDrive(const ros::NodeHandle& np): 
    RCDevice(np),
    cmd_var_(&position_),
    cmd_conv_to_(&pos_conv_to_),
    new_state_(false)
{
    np.param("joint_name", joint_name_, std::string("picdrive_joint"));

    double pos_conv_ratio, vel_conv_ratio, tqe_conv_ratio;
    std::string cmd_var;
    np.param("position_conv_ratio", pos_conv_ratio, 1.0);
    np.param("velocity_conv_ratio", vel_conv_ratio, 1.0);
    np.param("torque_conv_ratio", tqe_conv_ratio, 1.0);
    np.param("command_variable", cmd_var, std::string("position"));

    pos_conv_to_ = pos_conv_ratio;
    pos_conv_from_ = 1.0 / pos_conv_ratio;
    vel_conv_to_ = vel_conv_ratio;
    vel_conv_from_ = 1.0 / vel_conv_ratio;
    tqe_conv_to_ = tqe_conv_ratio;
    tqe_conv_from_ = 1.0 / tqe_conv_ratio;

    // TODO: admittance params
    // np.param("admittance_conv_to", admit_conv_to_, 64.0);
    // Converted tu uint16 at update time.
    //np.param("admittance_k", admit_k_, 0.0);
    //np.param("admittance_b", admit_b_, 0.0);
    //np.param("admittance_m", admit_m_, 0.0);


    // TODO: We could pick this up from the drive instead.
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
}

PICDrive::~PICDrive()
{
}

RCDevicePtr PICDrive::create(const ros::NodeHandle& np)
{
    return RCDevicePtr(new PICDrive(np));
}

void PICDrive::registerCtrlIfaces(IRLRobot& robot)
{
    hardware_interface::JointStateHandle sh(joint_name_, 
                                            &position_,
                                            &velocity_, 
                                            &torque_);
    robot.jsi().registerHandle(sh);

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

ThrottlingDef PICDrive::throttled(const TimeBase& p) const
{
    // UniDriveV2 limited to 1 message every 1 ms, meaning the maximum
    // state update frequency is 250 Hz (4 messages: 4 ms).
    ThrottlingDef td = {TimeBase(1000), 1};
    return td;
}

CANRobotDevice::State PICDrive::state() const
{
    return CANRobotDevice::state();
}

void PICDrive::enable(CANManager& can)
{
    CANRobotDevice::state(STATE_ENABLED);

    ROS_DEBUG("Enabling device %i", deviceID());

    position_      = 0.0;
    velocity_      = 0.0;
    torque_        = 0.0;
    set_point_     = *cmd_var_;
}

void PICDrive::enableCtrl(CANManager& can)
{
    ROS_INFO("Enabling control on device %i", deviceID());
    CANRobotDevice::enableCtrl(can);

    // Start the motor:
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
}

void PICDrive::disableCtrl(CANManager& can)
{
    CANRobotDevice::disableCtrl(can);

    // Stop the motor:
    LaboriusMessage msg;
    msg.msg_priority    = 0;
    msg.msg_type        = CAN_TYPE_ACTUATOR_HIGH_PRIORITY;
    msg.msg_cmd         = 0x80;
    msg.msg_boot        = 0;
    msg.msg_dest        = deviceID();
    msg.msg_data_length = 1;
    msg.msg_data[0]     = 0;
    msg.msg_remote      = 1;
    can.pushOneMessage(msg);
}

void PICDrive::disable(CANManager& can)
{
    CANRobotDevice::disable(can);
}

void PICDrive::requestState(CANManager& can)
{
    new_state_ = false;
}

bool PICDrive::stateReady()
{
    return new_state_;
}

void PICDrive::processMsg(const LaboriusMessage& msg)
{
    if (msg.msg_type == CAN_TYPE_ACTUATOR_HIGH_PRIORITY &&
        msg.msg_cmd == CAN_OUTPUT_DRIVE_STATE1) {

        int16_t torque   = *((int16_t*)(&msg.msg_data[0]));
        int16_t velocity = *((int16_t*)(&msg.msg_data[2]));
        int32_t position = *((int32_t *)(&msg.msg_data[4]));

        torque_   = torque   * tqe_conv_from_;
        velocity_ = velocity * vel_conv_from_;
        position_ = position * pos_conv_from_;

        new_state_ = true;
    }

}

void PICDrive::sendCommand(CANManager& can)
{
    if (state() != STATE_CONTROL) {
        return;
    }

    ROS_DEBUG_THROTTLE(1.0,
                       "Set point for %i: %f",
                       deviceID(),
                       set_point_);

    short cmd_s = set_point_ * *cmd_conv_to_;

    LaboriusMessage msg;
    msg.msg_cmd         = CAN_CMD_SETPOINT;
    msg.msg_type        = CAN_TYPE_ACTUATOR_HIGH_PRIORITY;
    msg.msg_priority    = 0;
    msg.msg_boot        = 0;
    msg.msg_remote      = 0;
    msg.msg_dest        = deviceID();
    msg.msg_data_length = 2;
    msg.msg_data[0]     = 0xFF & cmd_s;
    msg.msg_data[1]     = 0xFF & (cmd_s >> 8);

    can.pushOneMessage(msg);
}
