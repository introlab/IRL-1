#include <irl_can_ros_ctrl/uni_drive_v2.hpp>
#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <irl_can_bus/can_base_macros.h>
#include <ros/ros.h>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

namespace {
    static RCDeviceCreatorHelper ch_("UniDriveV2", 
                                     RCDeviceCreator(&UniDriveV2::create));
}

UniDriveV2::UniDriveV2(const ros::NodeHandle& np): 
    RCDevice(np),
    cmd_var_(&position_),
    cmd_conv_to_(&pos_conv_to_),
    cmd_var_type_(CMD_VAR_POSITION),
    set_point_(0.0),
    admittance_k_(0.0),
    admittance_b_(0.0),
    admittance_m_(0.0),
    polling_(true)
{
    np.param("joint_name", joint_name_, std::string("unidrive_v2_joint"));

    std::string cmd_var;
    np.param("command_variable", cmd_var, std::string("position"));

    // TODO: Get/Set this from/to the drive.
    if (cmd_var == "position")
    {
        cmd_var_        = &position_;
        cmd_conv_to_    = &pos_conv_to_;
        cmd_var_type_   = CMD_VAR_POSITION;
    }
    else if (cmd_var == "velocity")
    {
        cmd_var_        = &velocity_;
        cmd_conv_to_    = &vel_conv_to_;
        cmd_var_type_   = CMD_VAR_VELOCITY; 
    }
    else if (cmd_var == "torque")
    {
        cmd_var_        = &torque_;
        cmd_conv_to_    = &tqe_conv_to_;
        cmd_var_type_   = CMD_VAR_TORQUE; 
    }

    np.param("polling", polling_, true);

    // Create a new node handle to create a service server:
    ros::NodeHandle n(np);
    srv_admittance_ = 
        n.advertiseService("set_admittance", 
        &UniDriveV2::admittanceCB, this);
}

UniDriveV2::~UniDriveV2()
{
}

RCDevicePtr UniDriveV2::create(const ros::NodeHandle& np)
{
    return RCDevicePtr(new UniDriveV2(np));
}

void UniDriveV2::registerCtrlIfaces(IRLRobot& robot)
{
    // State interface
    hardware_interface::JointStateHandle sh(joint_name_, 
                                            &position_,
                                            &velocity_, 
                                            &torque_);
    robot.jsi().registerHandle(sh);

    // Command interface
    hardware_interface::JointCommandInterface* jci = nullptr;

    if (cmd_var_type_ == CMD_VAR_POSITION)
    {
        using HWI = hardware_interface::PositionJointInterface;
        jci = robot.getHWI<HWI>();
    }    
    else if (cmd_var_type_ == CMD_VAR_VELOCITY)
    {
        using HWI = hardware_interface::VelocityJointInterface;
        jci = robot.getHWI<HWI>();
    }
    else if (cmd_var_type_ == CMD_VAR_TORQUE)
    {
        using HWI = hardware_interface::EffortJointInterface;
        jci = robot.getHWI<HWI>();
    }
    
 
    if (jci != nullptr) {
        jci->registerHandle(hardware_interface::JointHandle(sh,
                                                            &set_point_));
    }
    else
    {
        ROS_ERROR("JointCommandInterface unknown for device : %i",deviceID());
    }
}

ThrottlingDef UniDriveV2::throttled(const TimeBase& p) const
{
    // UniDriveV2 limited to 1 message every 1 ms, meaning the maximum
    // state update frequency is 250 Hz (4 messages: 4 ms).
    ThrottlingDef td = {TimeBase(1000), 1};
    return td;
}

CANRobotDevice::State UniDriveV2::state() const
{
    return CANRobotDevice::state();
}

void UniDriveV2::enable(CANManager& can)
{
    req_state_ = STATE_ENABLED;
    CANRobotDevice::state(STATE_STARTING);

    position_      = 0.0;
    velocity_      = 0.0;
    torque_        = 0.0;
    torque_offset_ = 0.0;
    pos_offset_    = 0.0;

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

void UniDriveV2::enableCtrl(CANManager& can)
{
    req_state_ = STATE_CONTROL;
    CANRobotDevice::enableCtrl(can);

    // Start motor.
    unsigned char mode = MODE_VARIABLE_NORMAL;
    can.writeMem(deviceID(), MODE_VARIABLE_OFFSET, &mode, sizeof(unsigned char));
}

void UniDriveV2::disableCtrl(CANManager& can)
{
    req_state_ = STATE_ENABLED;
    CANRobotDevice::disableCtrl(can);

    // Stop motor.
    unsigned char mode = MODE_VARIABLE_IDLE;
    can.writeMem(deviceID(), MODE_VARIABLE_OFFSET, &mode, sizeof(unsigned char));
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
    if (new_state_ == ALL_RECEIVED) {
        return true;
    } else {
        return false;
    }
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
            drive_mode_ = msg.msg_data[0];
            // new_state_ |= MOD_RECEIVED;

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
            timebase_ = UNIDRIVE_V2_TIMEBASE_CONV_FROM *
                *((unsigned short*)msg.msg_data);
            ready_ |= TIMEBASE;
        break;
        
        case DRIVE_STATE_OFFSET:
            drive_state_ = *((unsigned short*)msg.msg_data);  
            new_state_ |= STA_RECEIVED;

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

    //ROS_WARN("Received, new_state: %i", new_state_);
    if (state() == STATE_STARTING) {
        //ROS_WARN("Received, ready: %i (conv: %i).", ready_, CONV_READY);
        if (ready_ == CONV_READY) {
            // All ratios were received, ready to convert.
            calcConvRatios();

            ROS_DEBUG("Switching UniDriveV2 %i to ENABLED.", deviceID());
            //ROS_WARN("Switching to ENABLED");
            CANRobotDevice::state(STATE_ENABLED);
        }
    }

}

void UniDriveV2::calcConvRatios()
{
    pos_conv_to_ = 1.0 / pos_conv_from_;	
    vel_conv_to_ = pos_conv_to_ * timebase_;
    vel_conv_from_ = 1.0 / vel_conv_to_;
    tqe_conv_to_ = 1.0 / tqe_conv_from_;
}

void UniDriveV2::sendCommand(CANManager& can)
{
    int setPointConv = 0;

    if (cmd_var_type_ == CMD_VAR_POSITION)
    {
        setPointConv = (int) *cmd_conv_to_ * set_point_;
    }
    else if (cmd_var_type_ == CMD_VAR_VELOCITY)
    {
        setPointConv = (int) *cmd_conv_to_ * set_point_;
    }
    else if (cmd_var_type_ == CMD_VAR_TORQUE)
    {
        setPointConv = (int) *cmd_conv_to_ * set_point_;
    }
    else
    {
        ROS_ERROR("Unhandled cmd_var_type_ : %i",cmd_var_type_);
        return;
    }

    ROS_DEBUG_THROTTLE(0.5,
                       "Dev: %i, SendCommand : %f (%i), mode: %i",
                       deviceID(), 
                       set_point_,
                       setPointConv,
                       drive_mode_);

    if (drive_mode_ == 1)
    {
        //ROS_INFO("Dev: %i, SendCommand : %f (%i)",deviceID(), set_point_,setPointConv);
        
        can.writeMem(deviceID(),
                     SETPOINT_VARIABLE_OFFSET, 
                     (unsigned char*) &setPointConv, 
                     sizeof(int)); 
        
    }

    if (admittance_changed_)
    {
        setAdmittance(can);
        admittance_changed_ = false;
    }

}

bool UniDriveV2::admittanceCB(SetAdmittance::Request& req,
                              SetAdmittance::Response&)
{
    ROS_INFO("admittanceCB : M: %f B: %f K: %f",req.m, req.b, req.k);
    //TODO VERIFY MIN/MAX RANGE
    admittance_m_ = req.m;
    admittance_b_ = req.b;
    admittance_k_ = req.k;
    admittance_changed_ = true;
    return true;
}

void UniDriveV2::setAdmittance(CANManager& can)
{  
    ROS_DEBUG("Changing impedance parameters for dev: %i, M: %f, B: %f, K: %f",
              deviceID(),
              admittance_m_,
              admittance_b_,
              admittance_k_);

    // TODO: VERIFY MIN/MAX RANGE

    can.writeMem(deviceID(),ADMITTANCE_M_OFFSET, (unsigned char*)&admittance_m_, sizeof(float));
    can.writeMem(deviceID(),ADMITTANCE_B_OFFSET, (unsigned char*)&admittance_b_, sizeof(float));
    can.writeMem(deviceID(),ADMITTANCE_K_OFFSET, (unsigned char*)&admittance_k_, sizeof(float));
}
