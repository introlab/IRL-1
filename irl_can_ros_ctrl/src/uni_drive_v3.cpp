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
    nh_(np),
    cmd_var_type_(CMD_VAR_POSITION),
    cmd_var_(&position_[TRANS_DATA_INDEX]),
    cmd_conv_to_(&pos_conv_to_),
    polling_(true),
    admittance_m_(0.0),
    admittance_b_(0.0),
    admittance_k_(0.0),
    admittance_changed_(false),
    pos_conv_to_(1.0),
    pos_conv_from_(1.0),
    vel_conv_to_(1.0),  
    vel_conv_from_(1.0), 
    tqe_conv_to_(1.0),
    tqe_conv_from_(1.0),
    transmission_pos_(1.0), //DEFAULT TRANSMISSION 1:1
    transmission_vel_(1.0), //DEFAULT TRANSMISSION 1:1
    transmission_eff_(1.0)  //DEFAULT TRANSMISSION 1:1
{
    np.param("joint_name", joint_name_, std::string("unidrive_v3_joint"));

    std::string cmd_var;
    np.param("command_variable", cmd_var, std::string("position"));

    //TRANSMISSIONS...
    double transmission_ratio = 1.0;
    np.param("pos_transmission_ratio", transmission_ratio, 1.0);
    transmission_pos_ = transmission_interface::SimpleTransmission(transmission_ratio);

    np.param("vel_transmission_ratio", transmission_ratio, 1.0);
    transmission_vel_ = transmission_interface::SimpleTransmission(transmission_ratio);

    np.param("eff_transmission_ratio", transmission_ratio, 1.0);
    transmission_eff_ = transmission_interface::SimpleTransmission(transmission_ratio);


    // TODO: Get/Set this from/to the drive.
    if (cmd_var == "position")
    {
        cmd_var_ = &position_[TRANS_DATA_INDEX];
        cmd_conv_to_ = &pos_conv_to_;
        cmd_var_type_ = CMD_VAR_POSITION;
    }
    else if (cmd_var == "velocity")
    {
        cmd_var_ = &velocity_[TRANS_DATA_INDEX];
        cmd_conv_to_ = &vel_conv_to_;
        cmd_var_type_ = CMD_VAR_VELOCITY;
    }
    else if (cmd_var == "torque")
    {
        cmd_var_ = &torque_[TRANS_DATA_INDEX];
        cmd_conv_to_ = &tqe_conv_to_;
        cmd_var_type_ = CMD_VAR_TORQUE;
    }

    np.param("polling", polling_, true);
    ROS_INFO("Joint name : %s, type: %s, polling: %i ratios: %f, %f, %f",
        joint_name_.c_str(), 
        cmd_var.c_str(), 
        polling_, 
        transmission_pos_.getActuatorReduction() ,
        transmission_vel_.getActuatorReduction() ,
        transmission_eff_.getActuatorReduction() );

    //Create services     
    admittance_changed_ = false;
 
    srv_admittance_ = 
        nh_.advertiseService("set_admittance", 
        &UniDriveV3::admittanceCB, this);


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

    //TRANSMISSION CONVERSION (RAW DATA)    
    actuator_data_.position.push_back(&position_[RAW_DATA_INDEX]);
    actuator_data_.velocity.push_back(&velocity_[RAW_DATA_INDEX]);
    actuator_data_.effort.push_back(&torque_[RAW_DATA_INDEX]);

    joint_data_.position.push_back(&position_[TRANS_DATA_INDEX]);
    joint_data_.velocity.push_back(&velocity_[TRANS_DATA_INDEX]);
    joint_data_.effort.push_back(&torque_[TRANS_DATA_INDEX]);

    act_to_jnt_pos_.registerHandle(transmission_interface::ActuatorToJointPositionHandle("trans_position", &transmission_pos_, actuator_data_, joint_data_));
    act_to_jnt_vel_.registerHandle(transmission_interface::ActuatorToJointVelocityHandle("trans_velocity", &transmission_vel_, actuator_data_, joint_data_));
    act_to_jnt_eff_.registerHandle(transmission_interface::ActuatorToJointEffortHandle("trans_effort", &transmission_eff_, actuator_data_, joint_data_));


    //JOINT STATE INTERFACE
    //AFTER TRANSMISSION...
    hardware_interface::JointStateHandle sh(joint_name_, 
                                            &position_[TRANS_DATA_INDEX],
                                            &velocity_[TRANS_DATA_INDEX], 
                                            &torque_[TRANS_DATA_INDEX]);
    robot.jsi().registerHandle(sh);


    //JOINT COMMAND INTERFACE    
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

    position_[RAW_DATA_INDEX]      = 0.0;
    velocity_[RAW_DATA_INDEX]      = 0.0;
    torque_[RAW_DATA_INDEX]        = 0.0;

    position_[TRANS_DATA_INDEX]      = 0.0;
    velocity_[TRANS_DATA_INDEX]      = 0.0;
    torque_[TRANS_DATA_INDEX]        = 0.0;


    torque_offset_ = 0.0;
    pos_offset_    = 0.0;


  

    admittance_changed_ = false;
    pos_offset_changed_ = false;

    ready_ = NONE_READY; 

    can.requestMem(deviceID(), POSITION_TO_RAD_VARIABLE_OFFSET, sizeof(float));
    can.requestMem(deviceID(), TORQUE_TO_NM_VARIABLE_OFFSET,    sizeof(float));
    can.requestMem(deviceID(), TIMEBASE_VARIABLE_OFFSET,        sizeof(short));
    can.requestMem(deviceID(), MAX_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), MIN_SETPOINT_VARIABLE_OFFSET,    sizeof(int));
    can.requestMem(deviceID(), POSITION_OFFSET_OFFSET,          sizeof(int));

    // Start the motor:
    // ENABLE MOTOR CTRL_MODE = 1, Will do init phase
    unsigned char mode = 1;
    can.writeMem(deviceID(), MODE_VARIABLE_OFFSET, &mode, sizeof(unsigned char));   
}

void UniDriveV3::enableCtrl(CANManager& can)
{
    req_state_ = STATE_CONTROL;
    CANRobotDevice::enableCtrl(can);
    ROS_INFO("enableCtrl %i",deviceID());   

    //SETPOINT = CURRENT VALUE
    set_point_     = *cmd_var_;
}

void UniDriveV3::disableCtrl(CANManager& can)
{
    req_state_ = STATE_ENABLED;
    CANRobotDevice::disableCtrl(can);
    ROS_INFO("disableCtrl %i",deviceID());  
}

void UniDriveV3::disable(CANManager& can)
{
    ROS_INFO("disable %i",deviceID());  
    req_state_ = STATE_DISABLED;
    CANRobotDevice::disable(can);

    // Stop motor.
    // ENABLE MOTOR CTRL_MODE = 0
    unsigned char mode = 0;
    can.writeMem(deviceID(), MODE_VARIABLE_OFFSET, &mode, sizeof(unsigned char)); 
}

void UniDriveV3::requestState(CANManager& can)
{

    //ROS_INFO("Drive: %i STATE: 0x%2.2x",deviceID(),new_state_);
    new_state_ = 0;

    // In non-polling mode, messages should come by themselves.
    // Do not send an extra request.
    if (polling_)
    {
        can.requestMem(deviceID(), POSITION_VARIABLE_OFFSET, sizeof(int));
        can.requestMem(deviceID(), SPEED_VARIABLE_OFFSET,    sizeof(int));
        can.requestMem(deviceID(), TORQUE_VARIABLE_OFFSET,   sizeof(int)); 
    }

    //Those variables need to be polled all the time
    can.requestMem(deviceID(), DRIVE_STATE_OFFSET,       sizeof(short));
    can.requestMem(deviceID(), MODE_VARIABLE_OFFSET,     sizeof(unsigned char));

    //last_request_ = ros::Time::now();

    //ROS_INFO("requestState : id: %i",deviceID());

}

bool UniDriveV3::stateReady()
{
    //ROS_INFO("DEVICE: %i, STATE_READY : %i",deviceID(), new_state_ == ALL_RECEIVED);

    if (new_state_ == ALL_RECEIVED) 
    {
        //Actuator to joint propagation
        if (drive_mode_ == 1)
        {
            act_to_jnt_pos_.propagate();
            act_to_jnt_vel_.propagate();
            act_to_jnt_eff_.propagate();
            return true;
        }
        ROS_WARN_THROTTLE(1.0,"DEVICE: %i E-STOP ENABLED ? CURRENT MODE : %i", deviceID(),drive_mode_);                 
    }
    else
    {
         ROS_WARN_THROTTLE(1.0,"NOT ALL_RECEIVED from %i",deviceID());
    } 
      
    //Not ready
    return false;
}

void UniDriveV3::calcConvRatios()
{
    pos_conv_to_ = 1.0 / pos_conv_from_;	
    vel_conv_to_ = pos_conv_to_ * timebase_;
    vel_conv_from_ = 1.0 / vel_conv_to_;
    tqe_conv_to_ = 1.0 / tqe_conv_from_;
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
            drive_mode_ = msg.msg_data[0];
            new_state_ |= MOD_RECEIVED;
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
                velocity_[RAW_DATA_INDEX] = vel_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= VEL_RECEIVED;
            }
        break;
        
        case POSITION_VARIABLE_OFFSET:
            if (ready_ == CONV_READY)
            {
                position_[RAW_DATA_INDEX] = pos_conv_from_ * *((int*)msg.msg_data);
                new_state_ |= POS_RECEIVED;
            }
        break;

        case TORQUE_VARIABLE_OFFSET:
            if (ready_ == CONV_READY)
            {
                torque_[RAW_DATA_INDEX] = tqe_conv_from_ * *((int*)msg.msg_data);
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
        //ROS_WARN("Received, ready: %i (conv: %i).", ready_, CONV_READY);
        if (ready_ == CONV_READY) {
            
            //MANDATORY IF WE WANT THE RIGHT UNITS...
            calcConvRatios();

            ROS_WARN("Switching UniDriveV3 %i to ENABLED.", deviceID());
            CANRobotDevice::state(STATE_ENABLED);
        }
    }

}

void UniDriveV3::sendCommand(CANManager& can)
{
    
    //MUST USE cmd_conv_to_ * set_point_ * transmission_ratio;
    //cmd_conv_to_ is initialized according to ratios and type (position, vel, eff)
    //set_point_ is modified by the command interface
    
    int setPointConv = 0;

    if (cmd_var_type_ == CMD_VAR_POSITION)
    {
        setPointConv = (int) (transmission_pos_.getActuatorReduction() *  *cmd_conv_to_ * set_point_);
    }
    else if (cmd_var_type_ == CMD_VAR_VELOCITY)
    {
        setPointConv = (int) (transmission_vel_.getActuatorReduction() *  *cmd_conv_to_ * set_point_);
    }
    else if (cmd_var_type_ == CMD_VAR_TORQUE)
    {
        setPointConv = (int) (transmission_eff_.getActuatorReduction() *  *cmd_conv_to_ * set_point_);
    }
    else
    {
        ROS_ERROR("Unhandled cmd_var_type_ : %i",cmd_var_type_);
        return;
    }

    //ROS_INFO_THROTTLE(0.5,"Dev: %i, SendCommand : %f (%i)",deviceID(), set_point_,setPointConv);
    if (drive_mode_ == 1)
    {
        //ROS_INFO("Dev: %i, SendCommand : %f (%i)",deviceID(), set_point_,setPointConv);
        can.writeMem(deviceID(), SETPOINT_VARIABLE_OFFSET, (unsigned char*) &setPointConv, sizeof(int)); 
    }

    if (admittance_changed_)
    {
        setAdmittance(can);
        admittance_changed_ = false;
    }

}

bool UniDriveV3::admittanceCB(SetAdmittance::Request& req, SetAdmittance::Response&)
{
    ROS_INFO("admittanceCB : M: %f B: %f K: %f",req.m, req.b, req.k);
    //TODO VERIFY MIN/MAX RANGE
    admittance_m_ = req.m;
    admittance_b_ = req.b;
    admittance_k_ = req.k;
    admittance_changed_ = true;
    return true;
}


void UniDriveV3::setAdmittance(CANManager& can)
{  
    ROS_INFO("Changing Admittance for dev: %i, M: %f, B: %f, K: %f",deviceID(),admittance_m_,admittance_b_,admittance_k_);
    //TODO VERIFY MIN/MAX RANGE
    can.writeMem(deviceID(),ADMITTANCE_M_OFFSET, (unsigned char*)&admittance_m_, sizeof(float));
    can.writeMem(deviceID(),ADMITTANCE_B_OFFSET, (unsigned char*)&admittance_b_, sizeof(float));
    can.writeMem(deviceID(),ADMITTANCE_K_OFFSET, (unsigned char*)&admittance_k_, sizeof(float));
}


