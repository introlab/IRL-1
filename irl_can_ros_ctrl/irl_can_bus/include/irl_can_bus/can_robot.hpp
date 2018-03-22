#ifndef CAN_ROBOT_HPP
#define CAN_ROBOT_HPP

#include "can_robot_device.hpp"
#include <functional>

namespace irl_can_bus
{
    /// \brief A class managing a collection of CAN devices through a single 
    /// CANManager.
    ///
    /// Note a CANRobot can only have a maximum of 256 devices - this is limited
    /// by the CAN id of 8-bit.
    ///
    /// The overall control loop should look like this (the first and normal
    /// loop runs are implemented in loop()):
    /// 
    /// Startup: 
    ///  - addDevice() for each device CANRobot should monitor.
    /// First loop:
    ///  - CANRobotDevice::enable() for each device.
    /// Normal loop:
    ///  - CANRobotDevice::requestState() for enabled devices.
    ///  - do {
    ///       CANManager::waitForMessages();
    ///       while ( CANManager::popOneMessage(msg) ) {
    ///           CANRobotDevice::processMessage(msg); 
    ///       }
    ///    } while (!deviceReady()) // On each enabled device.
    ///  - Call the robot control callbacks (see registerCtrlCB())
    ///  - CANRobotDevice::sendCommand() on each device in CONTROL state.
    ///    
    class CANRobot
    {
    private:
        CANManager                                       can_;
        std::array<CANRobotDevicePtr, MAX_CAN_DEV_COUNT> devices_;

        bool running_;

        std::vector< std::function<void ()> > ctrl_cbs_;

    public:
        /// \brief Constructor.
        ///
        /// \param ifaces A vector of names of CAN interfaces to use. 
        CANRobot(const std::vector<std::string>& ifaces);

        ~CANRobot();

        /// \brief Return a reference to the CANManager for this robot.
        CANManager&       canManager()       { return can_; }
        const CANManager& canManager() const { return can_; }

        /// \brief Add a device to monitor.
        ///
        /// Adding a device with an already used id will overwrite it.
        ///
        /// \param dev A shared pointer to a CANRobotDevice.
        void addDevice(const CANRobotDevicePtr& dev);

        /// \brief Register a callback for control calculations when a state is
        /// ready.
        ///
        /// Callbacks are called from loopOnce() if a full state is available
        /// for every enabled device.
        /// Can be called multiple times, callbacks will be called in their
        /// order of registration.
        ///
        /// \param fun Your function callback
        void registerCtrlCB(std::function<void ()> fun)
        {
            ctrl_cbs_.push_back(fun);
        }

        /// \brief Class member function version of registerCtrlCB.
        template <class T>
        void registerCtrlCB(void (T::*fun)(), T* obj)
        {
            registerCtrlCB(std::bind(fun, obj));
        }

        /// \brief Start the robot loop, enables every registered device.
        void start();

        /// \brief Stops the robot.
        ///
        /// NOTE: For now, only call stop() on CANManager, which should wake
        ///       loopOnce() from waiting on messages.
        void stop();

        /// \brief Blocking loop on the robot.
        ///
        /// Implements the control loop described in the class documentation for
        /// one cycle.
        void loopOnce();

    };

}

#endif

