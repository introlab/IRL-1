#include <jn0_face_ctrl/face_ctrl_base.hpp>
#include <irl_can_bus/can_base_macros.h>

using namespace jn0_face_ctrl;

FaceCtrlBase::FaceCtrlBase(const ros::NodeHandle& np)
{
    eyes_origin_.m_floats[3] = 0.0;
    np.param("eyes_origin_x", eyes_origin_.m_floats[0], 0.06);
    np.param("eyes_origin_y", eyes_origin_.m_floats[1], 0.00);
    np.param("eyes_origin_z", eyes_origin_.m_floats[2], 0.09);
    np.param("eyes_spacing", eyes_spacing_, 0.04);
    np.param("eyes_tilt_range", eyes_tilt_range_, 0.698);
    np.param("eyes_pan_range", eyes_pan_range_, 0.698);
    np.param("eyes_tilt_min", eyes_tilt_min_, -0.698); 
    np.param("eyes_tilt_max", eyes_tilt_max_, 0.698); 
    np.param("eyes_pan_min", eyes_pan_min_, -0.698); 
    np.param("eyes_pan_max", eyes_pan_max_, 0.698); 
    np.param("mouth_safety", mouth_safety_, false);
}

void FaceCtrlBase::setPose(const jn0_face_msgs::FacePose& cmd)
{
    pose_ = cmd; 

    //Software protection for avoid TOP and BOTTOM servo to make a collision
    
    // Left side
    if (mouth_safety_)
    {
        float& top    = pose_.top_left_mouth;
        float& bottom = pose_.bottom_left_mouth;
        bool top_follow_bottom = false;

        if (top < bottom) {
            top = bottom;
            top_follow_bottom = true;
        }

        if (top < -0.5) {
            top = -0.5;
        }

        if (bottom > 0.5) {
            bottom = 0.5;
            if (top_follow_bottom == true) {
                top = bottom;
            }
        }
    }
    
    // Right side
    if (mouth_safety_)
    {
        float& top    = pose_.top_right_mouth;
        float& bottom = pose_.bottom_right_mouth;
        bool top_follow_bottom = false;
        if (top > bottom){
            top = bottom;
            top_follow_bottom = true;
        }

        if(top > 0.5)
            top = 0.5;
        if(bottom < -0.5){
            bottom = -0.5;
            if(top_follow_bottom == true)
                top = bottom;
        }
    }
}

void FaceCtrlBase::setEyesTarget(const geometry_msgs::Point& msg)
{
    // 1. Find the target relative to the eyes' reference point.
    tf::Point target(msg.x, msg.y, msg.z);
    target -= eyes_origin_;

    // 2. Find the tilt angle.
    double tilt = -1.0 * atan2(-target.z(), target.x());

    // 3. Find the pan angle for both eyes.
    double pan_left =
        atan2(target.y() - eyes_spacing_, target.x()); 
    double pan_right =
        atan2(target.y() + eyes_spacing_, target.x()); 

    // 4. Clamp all values.
    pan_left = std::max(eyes_pan_min_, pan_left);
    pan_left = std::min(eyes_pan_max_, pan_left);
    pan_right = std::max(eyes_pan_min_, pan_right);
    pan_right = std::min(eyes_pan_max_, pan_right);
    tilt = std::max(eyes_tilt_min_, tilt);
    tilt = std::min(eyes_tilt_max_, tilt);
    
    ROS_DEBUG("Eyes target: (%f, %f, %f), "
        "tilt: %f, pan_l: %f, pan_r: %f",
        target.x(), target.y(), target.z(), tilt, pan_left, pan_right);

    // 5. Convert values and overwrite pose.
    pose_.eyes_tilt     = float(tilt / eyes_tilt_range_);
    pose_.left_eye_pan  = float(pan_left / eyes_pan_range_);
    pose_.right_eye_pan = float(pan_right / eyes_pan_range_);

}

void FaceCtrlBase::generateMessages(std::vector<LaboriusMessage>& msgs, int id)
{
    LaboriusMessage msg_base;

    msg_base.msg_priority       = 0;
    msg_base.msg_type           = CAN_TYPE_ACTUATOR_HIGH_PRIORITY;
    msg_base.msg_dest           = id;
    msg_base.msg_cmd            = DRIVE_CMD_DRIVE_SETPOINT;
    msg_base.msg_boot           = 0;
    msg_base.msg_remote         = 0;
    msg_base.msg_data_length    = 4;

    msgSetData(msg_base, FP_LEFT_EYE_PAN,       pose_.left_eye_pan);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_RIGHT_EYE_PAN,      pose_.right_eye_pan);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_EYES_TILT,          pose_.eyes_tilt);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_RIGHT_BROW,         pose_.right_brow);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_LEFT_BROW,          pose_.left_brow);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_TOP_LEFT_MOUTH,     pose_.top_left_mouth);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_BOTTOM_LEFT_MOUTH,  pose_.bottom_left_mouth);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_TOP_RIGHT_MOUTH,    pose_.top_right_mouth);
    msgs.push_back(msg_base);
    msgSetData(msg_base, FP_BOTTOM_RIGHT_MOUTH, pose_.bottom_right_mouth);
    msgs.push_back(msg_base);
}

void FaceCtrlBase::msgSetData(LaboriusMessage& msg_base, int motor, float value)
{
    msg_base.msg_data[0] = motor;
    msg_base.msg_data[1] = DEFAULT_SPEED;
    uint16_t pos = convertAngle(value);
    msg_base.msg_data[3] = pos >> 8;
    msg_base.msg_data[2] = pos & 0xff;
}

uint16_t FaceCtrlBase::convertAngle(float val)
{
    if(val < -1.0)
        val = -1.0;
    else if (val > 1.0)
        val = 1.0;
    
    // The input range for the multi-servo microcontroller is 2000,
    // which corresponds to periods between 1100 us and 1900 us,
    // or +/- 40 degrees.
    val = val * 1000 + 1000;

    return uint16_t(val);

}
