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
