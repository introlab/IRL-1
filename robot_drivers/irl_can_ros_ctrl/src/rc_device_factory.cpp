#include <irl_can_ros_ctrl/rc_device_factory.hpp>

using namespace irl_can_ros_ctrl;

RCDeviceFactory& RCDeviceFactory::get()
{
    static RCDeviceFactory factory;

    return factory;
}

RCDevicePtr RCDeviceFactory::createDevice(const std::string&     name,
                                          const ros::NodeHandle& np)
{
    return get().create(name, np);
}

RCDevicePtr RCDeviceFactory::create(const std::string&     name,
                                    const ros::NodeHandle& np)
{
    RCDevicePtr ptr;
    if (map_.find(name) != map_.end()) {
        ptr = map_[name](np);
    } else {
        std::stringstream ss;
        int msize = map_.size();
        auto i = map_.begin();
        int c = 0;
        while (i != map_.end()) {
            ss << i->first.c_str();
            ++i;
            if (++c < msize) {
                ss << ", ";
            }
        }
        ROS_ERROR("Cannot find device type %s, "
                  "available devices are: %s",
                  name.c_str(),
                  ss.str().c_str());
    }
    return ptr;
}

void RCDeviceFactory::registerCreator(const std::string& name,
                          RCDeviceCreator    c)
{
    return get().reg(name, c);
}

void RCDeviceFactory::reg(const std::string& name,
                          RCDeviceCreator    c)
{
    map_[name] = c;
}
