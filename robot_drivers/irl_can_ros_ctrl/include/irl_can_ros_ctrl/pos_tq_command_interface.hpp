#ifndef POS_TQ_COMMAND_INTERFACE_HPP
#define POS_TQ_COMMAND_INTERFACE_HPP

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace irl_can_ros_ctrl
{
    using JointStateHandle  = hardware_interface::JointStateHandle;
    template <class T, class C>
    using HRManager         = hardware_interface::HardwareResourceManager<T, C>;

    using ClaimResources    = hardware_interface::ClaimResources;

    /// \brief A ros_control hardware interface for position control with torque
    ///        offset (usually for gravity compensation).
    ///
    class PosTqJointHandle: public JointStateHandle
    {
    private:
        double* cmd_pos_;
        double* tq_offset_;

    public:
        PosTqJointHandle():
            JointStateHandle(),
            cmd_pos_(nullptr),
            tq_offset_(nullptr) 
        {}

        PosTqJointHandle(const JointStateHandle& js,
                         double* cmd_pos,
                         double* tq_offset):
            JointStateHandle(js),
            cmd_pos_(cmd_pos),
            tq_offset_(tq_offset)
        {
            using namespace hardware_interface;

            if (!cmd_pos_) {
                throw HardwareInterfaceException(
                    "Null pointer given for pos. set point.");
            }
            if (!tq_offset_) {
                throw HardwareInterfaceException(
                    "Null pointer given for torque offset.");
            }
        }

        void setCommand(double cmd_pos, double tq_offset)
        {
            setCommandPosition(cmd_pos);
            setTorqueOffset(tq_offset);
        }

        void setCommandPosition(double cmd_pos)
        {
            assert(cmd_pos_);
            *cmd_pos_ = cmd_pos;
        }

        void setTorqueOffset(double tq_offset)
        {
            assert(tq_offset_);
            *tq_offset_ = tq_offset;
        }

        double getCommandPosition() const
        {
            assert(cmd_pos_);
            return *cmd_pos_;
        }

        double getTorqueOffset() const
        {
            assert(tq_offset_);
            return *tq_offset_;
        }

    };

    class PosTqJointInterface: public HRManager<PosTqJointHandle,
                                                ClaimResources>
    {
    };
}

#endif
