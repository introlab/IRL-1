#include <irl_can_ros_ctrl/rc_device.hpp>

using namespace irl_can_ros_ctrl;
using namespace irl_can_bus;

RCDevice::RCDevice(const ros::NodeHandle& np)
{
    int dev_id;
    np.param("can_device_id", dev_id, -1);
    CANRobotDevice::deviceID(dev_id);
}

RCDevice::~RCDevice()
{
}

