#ifndef RC_DEVICE_FACTORY_HPP
#define RC_DEVICE_FACTORY_HPP

#include "rc_device.hpp"
#include <ros/ros.h>

namespace irl_can_ros_ctrl
{
    using RCDeviceCreator = std::function<RCDevicePtr (const ros::NodeHandle&)>;

    class RCDeviceFactory
    {
    private:
        std::map<std::string, RCDeviceCreator> map_;

    public:
        static RCDeviceFactory& get();

        static RCDevicePtr createDevice(const std::string&     type,
                                        const ros::NodeHandle& np);

        static void        registerCreator(const std::string& type,
                                           RCDeviceCreator    c);
    private:
        RCDevicePtr create(const std::string&     type,
                           const ros::NodeHandle& np);
        void reg(const std::string& type, RCDeviceCreator c);
    };

    class RCDeviceCreatorHelper
    {
    public:
        RCDeviceCreatorHelper(const std::string& name, RCDeviceCreator c)
        {
            RCDeviceFactory::registerCreator(name, c);
        }
    };
}

#endif

