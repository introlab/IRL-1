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
    cycle_i_(-1),
    new_pose_(true)
{
    np.param("clock_divider", clock_divider_, 10);

    ros::NodeHandle n(np);
    sub_pose_ = n.subscribe("cmd_face", 1, &FaceCtrlRC::poseCB, this);
    sub_target_ = n.subscribe("eyes_target", 1, &FaceCtrlRC::targetCB, this);

    // Default eyes target: 10 m ahead.
    geometry_msgs::Point pt;
    pt.x = 10.0;
    pt.y =  0.0;
    pt.z =  0.0;
    target_buffer_.writeFromNonRT(pt);
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

    if (!new_pose_) {
        return;
    }

    FaceCtrlBase::setPose(*(pose_buffer_.readFromRT()));
    FaceCtrlBase::setEyesTarget(*(target_buffer_.readFromRT()));

    std::vector<irl_can_bus::LaboriusMessage> msgs;
    msgs.reserve(FP_SIZE);
    FaceCtrlBase::generateMessages(msgs, deviceID());
    for (const auto& m: msgs) {
        can.pushOneMessage(m);
    }

    new_pose_ = false;
}

void FaceCtrlRC::poseCB(const jn0_face_msgs::FacePose& msg)
{
    pose_buffer_.writeFromNonRT(msg);
    new_pose_ = true;
}

void FaceCtrlRC::targetCB(const geometry_msgs::Point& msg)
{
    target_buffer_.writeFromNonRT(msg);
    new_pose_ = true;
}

