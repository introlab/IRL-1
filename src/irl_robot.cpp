#include <irl_can_ros_ctrl/irl_robot.hpp>
#include <irl_can_ros_ctrl/rc_device_factory.hpp>

using namespace irl_can_ros_ctrl;

IRLRobot::IRLRobot(ros::NodeHandle& n, ros::NodeHandle& np)
{
    std::vector<std::string> ifaces;
    if (np.hasParam("ifaces")) {
        XmlRpc::XmlRpcValue ifnames_arr; 
        np.getParam("ifaces", ifnames_arr);
        if (ifnames_arr.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < ifnames_arr.size(); ++i) {
                XmlRpc::XmlRpcValue& ifname = ifnames_arr[i];
                if (ifname.getType() == XmlRpc::XmlRpcValue::TypeString) {
                    ifaces.push_back(std::string(ifname));
                } else {
                    ROS_WARN("~ifaces element is not a string.");
                }
            }
        } else {
            ROS_WARN("~ifaces is not an array.");
        }
    }

    if (ifaces.empty()) {
        ROS_WARN("Empty interfaces list (~ifaces parameter).");
    }

    can_robot_.reset(new irl_can_bus::CANRobot(ifaces));

    XmlRpc::XmlRpcValue dev_arr;
    if (np.getParam("devices", dev_arr)) {
        if (dev_arr.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < dev_arr.size(); ++i) {
                XmlRpc::XmlRpcValue& dev_name = dev_arr[i];
                if (dev_name.getType() == XmlRpc::XmlRpcValue::TypeString) {
                    addDevice(ros::NodeHandle(np, std::string(dev_name)));
                } else {
                    ROS_ERROR("~devices element is not a string.");
                }
            }
        } else {
            ROS_ERROR("~devices parameter is not an array.");
        }
    } else {
        ROS_WARN("~devices not defined!");
    }

    can_robot_->registerCtrlCB(std::bind(&IRLRobot::control, this));
    can_robot_->start();

    double p;
    np.param("period", p, 0.01);
    timer_ = np.createTimer(ros::Duration(p), &IRLRobot::timerCB, this);
}

void IRLRobot::addDevice(const ros::NodeHandle& np)
{
    std::string dev_type;
    if (!np.getParam("can_device_type", dev_type)) {
        ROS_ERROR("Namespace %s does not define a can_device_type parameter, "
                  "skipping.",
                  np.getNamespace().c_str());
        return;
    }

    if (!np.hasParam("can_device_id")) {
        ROS_ERROR("Namespace %s does not define a can_device_id parameter, "
                  "skipping.",
                  np.getNamespace().c_str());
        return;
    }

    RCDevicePtr dev = RCDeviceFactory::createDevice(dev_type, np);

    if (dev) {
        devices_.push_back(dev);
        can_robot_->addDevice(dev);
        dev->registerCtrlIfaces(*this);
    } else {
        ROS_ERROR("Error creating device from namespace %s.",
                  np.getNamespace().c_str());
    }
}

void IRLRobot::control()
{
}

void IRLRobot::timerCB(const ros::TimerEvent&)
{
    can_robot_->loopOnce();
}

