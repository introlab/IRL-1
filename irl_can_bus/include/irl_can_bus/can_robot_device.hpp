#ifndef CAN_ROBOT_DEVICE_HPP
#define CAN_ROBOT_DEVICE_HPP

#include "can_manager.hpp"
#include "laborius_message.hpp"
#include "throttling_def.hpp"
#include <chrono>

namespace irl_can_bus
{
    /// \brief An abstract class representing a single CAN device on a bus.
    ///
    /// The normal state progress of a CAN device is this:
    ///
    ///  - DISABLED:   In this state until enable() is called.
    ///  - STARTING:   Transition state between DISABLED and ENABLED for drivers
    ///                that require async feedback from the device itself.
    ///  - ENABLED:    The device is enabled and its state is expected to be
    ///                updated.
    ///  - CONTROL:    Control is enabled on the device.
    ///
    /// Methods to implement:
    ///
    ///  - throttled()    Return the number of messages that can be sent to this
    ///                   device per reference period.
    ///                   Only called when added to the CANRobot instance.
    ///  - state()        Return the current state of the device.
    ///                   The default implementation directly returns what has
    ///                   been set by the state(const State&) method.
    ///  - requestState() Called at the beginning of the control loop.
    ///                   Messages that requests CAN variables should be sent
    ///                   at this moment.
    ///  - stateReady()   Indicates to the manager if a full state for this
    ///                   device has been received.
    ///  - sendCommand()  Called at the end of the control loop.
    ///                   Messages setting new control values should be sent at
    ///                   this moment.
    ///  - enable()       Called to enable the device, state goes from DISABLED
    ///                   to ENABLED.
    ///                   A state is not requested until the device is enabled.
    ///  - enableCtrl()   Called to enable control of the device, state goes
    ///                   from ENABLED to CONTROL.
    ///                   Commands are not requested (sendCommand()) until the
    ///                   the device is enabled for control.
    ///                   State requests are still available.
    ///  - disableCtrl()  Called to disable control the device, state goes from
    ///                   CONTROL to ENABLED.
    ///                   Request for state should still be possible.
    ///  - disable()      Disable the device completely, state goes from ENABLED
    ///                   or CONTROL to DISABLED.
    ///  - processMsg()   Process a single message coming from the CAN bus.
    class CANRobotDevice
    {
    public:
        enum State
        {
            STATE_DISABLED = 0, // Nothing should be done on the CAN bus.
            STATE_STARTING = 1, // Optional state between DISABLED and ENABLED.
            STATE_ENABLED  = 2, // State are requested, but control is disabled.
            STATE_CONTROL  = 3  // Control is enabled.
        };

    private:
        int   dev_id_;
        State state_;

    public:
        /// \brief Constructor.
        ///
        /// \param dev_id CAN bus device id.
        CANRobotDevice(int dev_id = -1): 
            dev_id_(dev_id),
            state_(STATE_DISABLED)
        {}

        /// \brief Return the CAN bus device id.
        int deviceID() const { return dev_id_; }

        /// \brief Set the CAN bus device id.
        ///
        /// NOTE: Has no effect after the device has been added to the manager.
        void deviceID(int d_id) { dev_id_ = d_id; } 

        /// \brief Return a throttling definition for this device.
        ///
        /// A value of (or below) for any value indicates that throttling is not
        /// necessary for this device.
        /// This is default implementation.
        ///
        /// \param p Throttling reference period.
        virtual ThrottlingDef throttled(const TimeBase& p) const
        {
            ThrottlingDef d = {TimeBase(0),0};
            return d;
        }

        /// \brief Return the current state of the device.
        /// 
        /// Can be overriden to consider internal state.
        virtual CANRobotDevice::State state() const 
        {
            return state_;
        }

        /// \brief Called by the manager to produce state request messages.
        ///
        /// Only called on devices in the ENABLED or CONTROL state.
        ///
        /// The default implementation does nothing, as might be the case for
        /// devices that automatically broadcast their state. 
        ///
        /// \param can A reference to the CANManager to use to send messages.
        virtual void requestState(CANManager& can)
        {
        };

        /// \brief Return true if a full state has been made available after a
        ///        requestState() call.
        virtual bool stateReady() { return false; }

        /// \brief Called when the robot is ready to send command messages.
        ///
        /// Only called on devices in the CONTROL state.
        /// 
        /// \param can A reference to the CANManager to use to send messages.
        virtual void sendCommand(CANManager& can) {};

        /// \brief Enable the device.
        ///
        /// Sending messages on the CAN bus is allowed.
        /// The default implementation only changes the state of the device.
        /// 
        /// \param can A reference to the CANManager to use to send messages.
        virtual void enable(CANManager& can)
        {
            if (state() == STATE_DISABLED) 
                state(STATE_ENABLED);
        }

        /// \brief Disabled the device.
        ///
        /// Sending messages on the CAN bus is allowed.
        /// The default implementation only changes the state of the device.
        /// If called from the CONTROL state, disableCtrl() is called first.
        /// 
        /// \param can A reference to the CANManager to use to send messages.
        virtual void disable(CANManager& can)
        {
            if (state() == STATE_CONTROL)
                disableCtrl(can);

            state(STATE_DISABLED);
        }

        /// \brief Enable control of the device.
        ///
        /// Sending messages on the CAN bus is allowed.
        /// The default implementation only changes the state of the device.
        /// Does not skip steps if the device is in DISABLED state.
        /// 
        /// \param can A reference to the CANManager to use to send messages.
        virtual void enableCtrl(CANManager& can)
        {
            if (state() == STATE_ENABLED) 
                state(STATE_CONTROL);
        }

        /// \brief Disable control of the device.
        ///
        /// Sending messages on the CAN bus is allowed.
        /// The default implementation only changes the state of the device to
        /// ENABLED.
        /// 
        /// \param can A reference to the CANManager to use to send messages.
        virtual void disableCtrl(CANManager& can)
        {
            if (state() == STATE_CONTROL) 
                state(STATE_ENABLED);
        }
    
        /// \brief Process an incoming message.
        ///
        /// The default implementation does nothing.
        virtual void processMsg(const LaboriusMessage& msg) {}

    protected:
        /// \brief Set the device state.
        void state(const State& state) { state_ = state; } 

    };

    using CANRobotDevicePtr = std::shared_ptr<CANRobotDevice>;
}

#endif
