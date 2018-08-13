#include <jn0_face_ctrl/face_ctrl_rc.hpp>

using namespace jn0_face_ctrl;

namespace {
    static irl_can_ros_ctrl::RCDeviceCreatorHelper ch_(
        "FaceCtrlRC", 
        irl_can_ros_ctrl::RCDeviceCreator(&FaceCtrlRC::create));
}

RCDevicePtr FaceCtrlRC::create(const ros::NodeHandle& np)
{
    return RCDevicePtr(new FaceCtrlRC(np));
}

FaceCtrlRC::FaceCtrlRC(const ros::NodeHandle& np):
    RCDevice(np), 
    FaceCtrlBase(np),
    cycle_i_(-1)
{
    np.param("clock_divider", clock_divider_, 10);
}

void FaceCtrlRC::registerCtrlIfaces(irl_can_ros_ctrl::IRLRobot& robot)
{
    // This function left intentionally blank.
}

void FaceCtrlRC::processMsg(const irl_can_bus::LaboriusMessage& msg)
{
    // This function left intentionally blank.
}

void FaceCtrlRC::sendCommand(irl_can_bus::CANManager& can)
{
    cycle_i_++;
    if ((cycle_i_ % clock_divider_) != 0) {
        return;
    }
    cycle_i_ = -1;

    std::vector<irl_can_bus::LaboriusMessage> msgs;
    msgs.reserve(FP_SIZE);
    FaceCtrlBase::generateMessages(msgs, deviceID());
    for (const auto& m: msgs) {
        can.pushOneMessage(m);
    }
}
