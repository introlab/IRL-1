#include <irl_can_ros_ctrl/ptu_drive.hpp>
#include <irl_can_ros_ctrl/irl_robot.hpp>

using namespace irl_can_ros_ctrl;

namespace {
    static RCDeviceCreatorHelper ch_("PTUDrive", 
                                     RCDeviceCreator(&PTUDrive::create));
}

RCDevicePtr PTUDrive::create(const ros::NodeHandle& np)
{
    return RCDevicePtr(new PTUDrive(np));
}

PTUDrive::PTUDrive(const ros::NodeHandle& n):
    RCDevice(n),
    cur_pos_(0.0), 
    cur_vel_(0.0), 
    cur_eff_(0.0),
    cur_cmd_(0.0),
    state_ready_(false),
    cycle_(0)
{
    n.param("joint_name", joint_name_, std::string("ptu_joint"));
    n.param("motor_enabled", motor_enabled_, true);
    int boffset;
    n.param("base_offset", boffset, 0);
    base_offset_ = (unsigned char)(boffset & 0xFF);
    n.param("position_conv_ratio", pos_conv_to_, 195.3788);
    pos_conv_from_ = 1.0 / pos_conv_to_;
    n.param("position_center", pos_center_, 512);
    n.param("velocity_conv_ratio", vel_conv_from_, 0.0116);
    vel_conv_to_ = 1.0 / vel_conv_from_;
    n.param("max_velocity", max_vel_, 1.047);
    n.param("rest_angle", rest_angle_, 0.0);
    n.param("stop_at_shutdown", stop_at_shutdown_, true);
    n.param("clock_divider", clock_divider_, 10);

    ROS_INFO("Create PTU controller for %i:%i.", deviceID(), base_offset_);
}

void PTUDrive::registerCtrlIfaces(IRLRobot& robot)
{
    hardware_interface::JointStateHandle sh(joint_name_, 
                                            &cur_pos_,
                                            &cur_vel_, 
                                            &cur_eff_);
    robot.jsi().registerHandle(sh);

    using HWI = hardware_interface::PositionJointInterface;
    hardware_interface::JointCommandInterface* jci = robot.getHWI<HWI>();
    if (jci != nullptr) {
        jci->registerHandle(hardware_interface::JointHandle(sh,
                                                            &cur_cmd_));
    }
}

void PTUDrive::enable(irl_can_bus::CANManager& can)
{
    CANRobotDevice::state(STATE_STARTING);

    cur_pos_ = 0.0;
    cur_vel_ = 0.0;
    cur_eff_ = 0.0;
    cur_cmd_ = cur_pos_;

    setMaxVel(can);

    state_ready_ = false; // Will turn true after receiving the current state.
}

void PTUDrive::enableCtrl(irl_can_bus::CANManager& can)
{
    CANRobotDevice::enableCtrl(can);

    if (!motor_enabled_)
        ROS_WARN("Starting PTU driver with a disabled motor.");

    cycle_ = 0;
}

void PTUDrive::disableCtrl(irl_can_bus::CANManager& can)
{
    setPoint(can, rest_angle_);
    if (stop_at_shutdown_)
        stopMotor(can);

    CANRobotDevice::disableCtrl(can);
}

void PTUDrive::disable(irl_can_bus::CANManager& can)
{
    CANRobotDevice::disable(can);
}

void PTUDrive::processMsg(const irl_can_bus::LaboriusMessage& msg)
{
    unsigned int dest = msg.msg_cmd - base_offset_;

    //ROS_DEBUG(
    //    "Message from PTU drive, state: %i, dest: %i, offset: %i", 
    //    CANRobotDevice::state(),
    //    dest,
    //    base_offset_);

    switch (dest)
    {
        case CUR_POS_OFFSET:
            cur_pos_ = pos_conv_from_ * 
                (*((int16_t*)msg.msg_data) - pos_center_);
            if (CANRobotDevice::state() == STATE_STARTING)
            {
                // A valid position has been received, indicate that the
                // state is ready, and sendCommand can switch on the motor
                // and go to state 'CONTROL'.
                state_ready_ = true;
                cur_cmd_ = cur_pos_;
                CANRobotDevice::state(STATE_CONTROL);
            }
            break;
        case CUR_VEL_OFFSET:
            cur_vel_ = vel_conv_from_ *
                *((int16_t*)msg.msg_data);
            break;

        default:
            break;
    };

}

void PTUDrive::requestState(irl_can_bus::CANManager& can)
{
    // NOTE: cycle_ is updated in sendCommand() - they both have to be
    // in sync.
    if ((cycle_ % clock_divider_) == 0)
    {
        // We have to poll the drive periodically, it does not 
        // automatically broadcast its state.
        can.requestMem(deviceID(), base_offset_ + CUR_POS_OFFSET, sizeof(int16_t));
        can.requestMem(deviceID(), base_offset_ + CUR_VEL_OFFSET, sizeof(int16_t));
    }


}

void PTUDrive::sendCommand(irl_can_bus::CANManager& can)
{
    if ((cycle_++ % clock_divider_) != 0)
        return;
    if (cycle_ == std::numeric_limits<int>::max())
        cycle_ %= clock_divider_;

    if (CANRobotDevice::state() == STATE_CONTROL) {
        // Only apply set points in the CONTROL state, the STARTING state
        // waits for a valid position from the drive before switching to the
        // CONTROL state.
        setPoint(can, cur_cmd_);
    }
}

void PTUDrive::startMotor(irl_can_bus::CANManager& can)
{
    static unsigned char cmd = 1;
    can.writeMem(deviceID(), base_offset_ + TORQUE_ENABLE_OFFSET, &cmd, sizeof(char));
}

void PTUDrive::stopMotor(irl_can_bus::CANManager& can)
{
    static unsigned char cmd = 0;
    can.writeMem(deviceID(), base_offset_ + TORQUE_ENABLE_OFFSET, &cmd, sizeof(char));
}

void PTUDrive::setPoint(irl_can_bus::CANManager& can, double val)
{
    if (!motor_enabled_)
        return;
    int16_t cmd = int(pos_conv_to_ * val) + pos_center_;
    can.writeMem(deviceID(), base_offset_ + SET_POINT_OFFSET, 
        (unsigned char*)&cmd, sizeof(int16_t));

    ROS_DEBUG_THROTTLE(1.0, "PTU SetPoint for %i:%i: %i",
                       deviceID(), base_offset_, cmd);
}

void PTUDrive::setMaxVel(irl_can_bus::CANManager& can)
{
    if (!motor_enabled_)
        return;
    int16_t cmd = int(vel_conv_to_ * max_vel_);
    can.writeMem(deviceID(), base_offset_ + MOVING_SPEED_OFFSET,
        (unsigned char*)&cmd, sizeof(int16_t));
}

