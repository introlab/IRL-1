#include <controller_interface/controller.h>
#include <irl_can_ros_ctrl/pos_tq_command_interface.hpp>
#include <pluginlib/class_list_macros.h>
#include <jn0_arm_common/jn0_arm_common.hpp>

namespace jn0_arm_ctrl
{
    using PosTqJointInterface   = irl_can_ros_ctrl::PosTqJointInterface;
    template <class T>
    using Controller            = controller_interface::Controller<T>;

    class Jn0ArmController: public Controller<PosTqJointInterface>
    {
    private:
        std::vector<irl_can_ros_ctrl::PosTqJointHandle> joints_;
        
        std::unique_ptr<Jn0ArmCommon> arm_l_;
        std::unique_ptr<Jn0ArmCommon> arm_r_;

    public:
        Jn0ArmController():
            joints_(Jn0ArmCommon::NUM_JOINTS)
        {
        }

        bool init(irl_can_ros_ctrl::PosTqJointInterface* hw, ros::NodeHandle& np)
        {
            arm_l_.reset(new Jn0ArmCommon(Jn0ArmCommon::LEFT));
            arm_r_.reset(new Jn0ArmCommon(Jn0ArmCommon::RIGHT));

            const auto joint_names = arm_l_->getJointNames();

            for (int i = 0; i < Jn0ArmCommon::NUM_JOINTS; ++i) {
                joints_[i] = hw->getHandle(joint_names[i]);
            }
        }

    };
}


