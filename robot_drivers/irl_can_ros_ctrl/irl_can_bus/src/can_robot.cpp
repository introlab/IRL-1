#include <irl_can_bus/can_robot.hpp>

using namespace irl_can_bus;

const char* CANRobotDevice::StateNames[4] = {
            "DISABLED",
            "STARTING",
            "ENABLED",
            "CONTROL"
};

CANRobot::CANRobot(const std::vector<std::string>& ifaces):
    can_(ifaces)
{
}


CANRobot::~CANRobot()
{
    std::cerr<<"~CANRobot"<<std::endl;
    stop();
    std::cerr<<"~CANRobot done!"<<std::endl;  
}

void CANRobot::addDevice(const CANRobotDevicePtr& dev)
{
    int dev_id = dev->deviceID();
    if (dev_id <= MAX_CAN_DEV_ID) {
        devices_[dev_id] = dev;
    } else {
        CAN_LOG_ERROR("Attempted to add a device with an invalid ID: %i",
                      dev_id);
        return;
    }

    ThrottlingDef td = dev->throttled(can_.throttlingPeriod());
    if (td.valid()) {
        can_.throttling(dev_id, td);
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
    CAN_LOG_INFO("CANRobot::stop()");

    for (auto dev: devices_) {
        if (dev) {
            dev->disable(can_);
        }
    }
    running_ = false;
    can_.stop();    
    CAN_LOG_INFO("CANRobot::stop() done");
}

void CANRobot::loopOnce()
{
    ///Request state
    int  enabled_count = 0;
    for (auto& dev: devices_) {
        if (dev) {
            if (dev->state() != CANRobotDevice::STATE_DISABLED) 
            {
                // CAN_LOG_INFO("REQUESTING STATE FOR DEV : %i",dev->deviceID());
                dev->requestState(can_);
                ++enabled_count;
            }
        }
    }

    ///Read messages
    LaboriusMessage msg_in;
    while (can_.popOneMessage(msg_in)) {
        //CAN_LOG_INFO("Got one message from %i.",
        //              msg_in.msg_dest);
        CANRobotDevicePtr& dev = devices_[msg_in.msg_dest];
        if (dev && dev->state() != CANRobotDevice::STATE_DISABLED) {
            dev->processMsg(msg_in);
        }
    }

    bool all_ready = true;
    for (auto& dev: devices_) 
    {
        if (dev && ((dev->state() == CANRobotDevice::STATE_ENABLED)  || 
                    (dev->state() == CANRobotDevice::STATE_CONTROL))   ) 
        {
            if (!dev->stateReady()) 
            {
                all_ready = false;
                
                //CAN_LOG_WARN("NOT READY, Will request state again for %i.",
                //             dev->deviceID());

                if (dev->state() == CANRobotDevice::STATE_CONTROL) 
                {
                    dev->disableCtrl(can_);                   
                }
            } 
            else if (dev->state() == CANRobotDevice::STATE_ENABLED) 
            {
                CAN_LOG_DEBUG("READY, Enabling control on %i", dev->deviceID());
                dev->enableCtrl(can_);
            }
        }
    }
    
    if (all_ready)
    {
        for (const auto& cb : ctrl_cbs_) {
            cb();
        }
        
        for (auto dev: devices_) {
            if (dev && (dev->state() == CANRobotDevice::STATE_CONTROL)) {
                dev->sendCommand(can_);
            }
        }
    }
    else
    {
        //CAN_LOG_WARN("WAITING FOR ALL MODULES READY");        
    }

#if 0
    bool all_ready = false;
    bool timed_out = false;
    int cycles = 0;
    do {
        if (enabled_count > 0) {
            timed_out = !can_.waitForMessages();
        }

        if (timed_out) {
            CAN_LOG_ERROR("Timed out on waitForMessages!");
        }

        if (!running_)
            return;//return // Stop has been requested.

        LaboriusMessage msg_in;
        while (can_.popOneMessage(msg_in)) {
            CAN_LOG_WARN("Got one message from %i.",
                          msg_in.msg_dest);
            CANRobotDevicePtr& dev = devices_[msg_in.msg_dest];
            if (dev && dev->state() != CANRobotDevice::STATE_DISABLED) {
                dev->processMsg(msg_in);
            }
        }

        all_ready = true;
        for (auto& dev: devices_) {
            if (dev && ((dev->state() == CANRobotDevice::STATE_ENABLED)  || 
                        (dev->state() == CANRobotDevice::STATE_CONTROL))   ) {
                if (!dev->stateReady()) {
                    all_ready = false;
                    if (timed_out) {
                        CAN_LOG_WARN("NOT READY, Requesting state again for %i.",
                                     dev->deviceID());

                        dev->requestState(can_);
                        if (dev->state() == CANRobotDevice::STATE_CONTROL) {
                            dev->disableCtrl(can_);
                        }
                    } else {
                        break;
                    }
                } else if (dev->state() == CANRobotDevice::STATE_ENABLED) {
                    CAN_LOG_DEBUG("READY, Enabling control on %i", dev->deviceID());
                    dev->enableCtrl(can_);
                }
            }
        }

        // TODO: Detect if something has timed out.
        ++cycles;

        //if (cycles > 2)
          //  break;
    } while (!all_ready);

    if (cycles > 100) {
        CAN_LOG_DEBUG("cycles: %i", cycles);
    }

    for (const auto& cb : ctrl_cbs_) {
        cb();
    }
    
    for (auto dev: devices_) {
        if (dev && (dev->state() == CANRobotDevice::STATE_CONTROL)) {
            dev->sendCommand(can_);
        }
    }

    #endif
}

