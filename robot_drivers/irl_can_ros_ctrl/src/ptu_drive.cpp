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
    RCDevice(n)
{
    ros::NodeHandle n_pan(n, "pan_motor");
    ros::NodeHandle n_tilt(n, "tilt_motor");

    if (!n_pan.hasParam("can_device_id")) {
        n_pan.setParam("can_device_id", deviceID());
    }
    if (!n_tilt.hasParam("can_device_id")) {
        n_tilt.setParam("can_device_id", deviceID());
    }

    pan_motor_.reset(new PTUMotor(n_pan));
    tilt_motor_.reset(new PTUMotor(n_tilt));

}

void PTUDrive::registerCtrlIfaces(IRLRobot& robot)
{
    pan_motor_->registerCtrlIfaces(robot);
    tilt_motor_->registerCtrlIfaces(robot);
}

bool PTUDrive::stateReady()
{
    return pan_motor_->stateReady() &&
           tilt_motor_->stateReady();
}

void PTUDrive::enable(irl_can_bus::CANManager& can)
{
    CANRobotDevice::state(STATE_STARTING);

    pan_motor_->enable(can);
    tilt_motor_->enable(can);
}

void PTUDrive::enableCtrl(irl_can_bus::CANManager& can)
{
    CANRobotDevice::enableCtrl(can);

    pan_motor_->enableCtrl(can);
    tilt_motor_->enableCtrl(can);
}

void PTUDrive::disableCtrl(irl_can_bus::CANManager& can)
{
    pan_motor_->disableCtrl(can);
    tilt_motor_->disableCtrl(can);

    CANRobotDevice::disableCtrl(can);
}

void PTUDrive::disable(irl_can_bus::CANManager& can)
{
    pan_motor_->disable(can);
    tilt_motor_->disable(can);

    CANRobotDevice::disable(can);
}

void PTUDrive::processMsg(const irl_can_bus::LaboriusMessage& msg)
{
    //ROS_DEBUG_THROTTLE(1.0, "PTUDrive processMsg for %i", deviceID());
    pan_motor_->processMsg(msg);
    tilt_motor_->processMsg(msg);

    RCDevice::State combined_state = std::min(pan_motor_->state(),
                                              tilt_motor_->state());
    CANRobotDevice::state(combined_state);
    ROS_DEBUG_THROTTLE(1.0, "PTUDrive state: %s",
                       RCDevice::StateNames[state()]);
}

void PTUDrive::requestState(irl_can_bus::CANManager& can)
{
    ROS_DEBUG_THROTTLE(1.0, "PTUDrive requestState for %i", deviceID());
    pan_motor_->requestState(can);
    tilt_motor_->requestState(can);
}

void PTUDrive::sendCommand(irl_can_bus::CANManager& can)
{
    ROS_DEBUG_THROTTLE(1.0, "PTUDrive sendCommand for %i", deviceID());
    pan_motor_->sendCommand(can);
    tilt_motor_->sendCommand(can);
}

