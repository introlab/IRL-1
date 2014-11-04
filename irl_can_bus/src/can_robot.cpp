#include <irl_can_bus/can_robot.hpp>

using namespace irl_can_bus;

CANRobot::CANRobot(const std::vector<std::string>& ifaces):
    can_(ifaces)
{
}

void CANRobot::addDevice(const CANRobotDevicePtr& dev)
{
    int dev_id = dev->deviceID();
    if (dev_id <= MAX_CAN_DEV_ID) {
        devices_[dev_id] = dev;
    }
}

void CANRobot::start()
{
    for (auto dev: devices_) {
        if (dev) {
            dev->enable(can_);
        }
    }

    running_ = true;
}

void CANRobot::stop()
{
    CAN_LOG_DEBUG("CANRobot::stop()");
    can_.stop();
    running_ = false;
    CAN_LOG_DEBUG("CANRobot::stop() done");
}

void CANRobot::loopOnce()
{
    int  enabled_count = 0;
    for (auto dev: devices_) {
        if (dev && (dev->state() != CANRobotDevice::STATE_DISABLED)) {
            dev->requestState(can_);
            ++enabled_count;
        }
    }

    bool all_ready;
    do {
        can_.waitForMessages();

        if (!running_)
            return; // Stop has been requested.

        LaboriusMessage msg_in;
        while (can_.popOneMessage(msg_in)) {
            CAN_LOG_DEBUG("Got one message from %i.",
                          msg_in.msg_dest);
            CANRobotDevicePtr& dev = devices_[msg_in.msg_dest];
            if (dev && dev->state() != CANRobotDevice::STATE_DISABLED) {
                dev->processMsg(msg_in);
            }
        }

        all_ready = true;
        for (auto dev: devices_) {
            if (dev && (dev->state() == CANRobotDevice::STATE_ENABLED)) {
                if (!dev->stateReady()) {
                    all_ready = false;
                    break;
                }
            }
        }

        // TODO: Detect if something has timed out.
        
    } while (!all_ready);

    for (const auto& cb : ctrl_cbs_) {
        cb();
    }
    
    for (auto dev: devices_) {
        if (dev && (dev->state() == CANRobotDevice::STATE_CONTROL)) {
            dev->sendCommand(can_);
        }
    }
}

