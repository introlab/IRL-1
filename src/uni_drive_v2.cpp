#include <irl_can_ros_ctrl/uni_drive_v2.hpp>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

UniDriveV2::UniDriveV2(int dev_id): 
    CANRobotDevice(dev_id),
    pos_(0),
    pos_ready_(false)
{
}

void UniDriveV2::enable(CANManager& can)
{
    CANRobotDevice::enable(can);
}

void UniDriveV2::enableCtrl(CANManager& can)
{
    CANRobotDevice::enableCtrl(can);

}

void UniDriveV2::disableCtrl(CANManager& can)
{
    CANRobotDevice::disableCtrl(can);
}

void UniDriveV2::disable(CANManager& can)
{
    CANRobotDevice::disable(can);
}

void UniDriveV2::requestState(CANManager& can)
{
    using namespace irl_can_bus;

    LaboriusMessage msg_req;
    requestMem(msg_req, 
               deviceID(),
               POSITION_VARIABLE_OFFSET, 
               sizeof(int));
    can.pushOneMessage(msg_req);
    pos_ready_ = false;
}

bool UniDriveV2::stateReady()
{
    return pos_ready_;
}

void UniDriveV2::processMsg(const LaboriusMessage& msg)
{
    switch(msg.msg_cmd) {
        case POSITION_VARIABLE_OFFSET:
            pos_ = *((int*)msg.msg_data);
            pos_ready_ = true;
        break;

        default:
        break;
    };
}

void UniDriveV2::update()
{
    std::cout << "Pos: " << pos_ << std::endl;
}
