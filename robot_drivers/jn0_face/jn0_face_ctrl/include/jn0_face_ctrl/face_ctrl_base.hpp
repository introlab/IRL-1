#ifndef FACE_CTRL_BASE_HPP
#define FACE_CTRL_BASE_HPP

#include <jn0_face_msgs/FacePose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <irl_can_bus/laborius_message.hpp>
#include <vector>

namespace jn0_face_ctrl 
{
    typedef irl_can_bus::LaboriusMessage LaboriusMessage;

    /// \brief Base class for face expression controller.
    ///
    /// Performs basic pose transforms and CAN message generation.
    /// Does not directly interfaces with the outside (ROS or CAN) except for
    /// ROS parameters.
    ///
    /// Inputs:
    ///  - command: A FacePose message for the whole face.
    ///  - eyes_target: A Point message that acts as a focusing target
    ///    for the eyes.
    ///    The reference frame is as follows:
    ///     - X: Points outward the face.
    ///     - Y: Points toward the left.
    ///     - Z: Points toward the top of the head.
    ///     - Origin: At the head's parent (normally neck_top_frame) with
    ///       the same orientation.
    ///    The vector as to be pre-transformed, we refrain from running TF
    ///    transforms inside low-level controllers.
    ///    Calculation are performed according to the eyes origin, a point
    ///    situated in the middle of both eyes on the face's plane in the
    ///    head's parent frame.
    ///    Angles are limited by the min/max pan/tilt values, but are scaled
    ///    using the range parameters.
    ///
    ///    The normal operating procedure is to call setPose and setEyesTarget
    ///    first, then generateMessages to publish the new set points over the
    ///    CAN bus.
    ///
    /// Parameters:
    ///  - eyes_origin_x: Eyes reference point in X.
    ///    Default: 0.06 m.
    ///  - eyes_origin_y: Eyes reference point in Y.
    ///    Default: 0.00 m.
    ///  - eyes_origin_z: Eyes reference point in Z.
    ///    Default: 0.09 m. 
    ///  - eyes_spacing: Spacing (along z) between the reference point and the
    ///    eyes' centers.
    ///    Default: 0.04 m.
    ///  - eyes_tilt_range: Eyes tilt angle range.
    ///    Default: 0.698 rads.
    ///  - eyes_pan_range: Eyes pan range.
    ///    Default: 0.698 rads.
    ///  - eyes_tilt_min: Minimum tilt angle.
    ///    Defualt: -0.698 rads.
    ///  - eyes_tilt_max: Maximum tilt angle.
    ///    Default: 0.698 rads.
    ///  - eyes_pan_min: Minimum pan angle.
    ///    Default: -0.698 rads.
    ///  - eyes_pan_max: Maximum pan angle.
    ///    Default: 0.698 rads.
    ///
	class FaceCtrlBase
	{
    public:
        enum
        {
            FP_LEFT_EYE_PAN         = 0,
            FP_RIGHT_EYE_PAN        = 1,
            FP_EYES_TILT            = 2,
            FP_RIGHT_BROW           = 3,
            FP_LEFT_BROW            = 4,
            FP_BOTTOM_RIGHT_MOUTH   = 5,
            FP_TOP_LEFT_MOUTH       = 6,
            FP_BOTTOM_LEFT_MOUTH    = 7,
            FP_TOP_RIGHT_MOUTH      = 8,
            FP_SIZE                 = 9
        };

        static const unsigned int DEFAULT_SPEED = 60;
        static const unsigned int DRIVE_CMD_DRIVE_SETPOINT = 0x01;

    private:
        tf::Point   eyes_origin_;
        double      eyes_spacing_;
        double      eyes_tilt_range_;
        double      eyes_pan_range_;
        double      eyes_tilt_min_;
        double      eyes_tilt_max_;
        double      eyes_pan_min_;
        double      eyes_pan_max_;
        bool        mouth_safety_;

        jn0_face_msgs::FacePose pose_;

	public:
        /// \brief Default constructor.
        ///
        /// Does not result in a working driver.
        /// Note that pluginlib needs a defined default constructor to work.
        FaceCtrlBase() {}

        /// \brief Constructor.
        ///
        /// \param np Node handle for parameters.
		FaceCtrlBase(const ros::NodeHandle& np);

        virtual ~FaceCtrlBase() {};

        /// \brief Sets a new pose for the face.
        ///
        /// Note that the eyes angles can be overwritten by the setEyeTarget
        /// method, and vice versa.
        /// \param cmd The full pose. 
		void setPose(const jn0_face_msgs::FacePose& cmd);

        /// \brief Sets a new target for the eyes.
        ///
        /// Overwrites internal state of the eyes with the given target.
        ///
        /// \param cmd The new target point.
        void setEyesTarget(const geometry_msgs::Point& cmd);

        /// \brief Generates new commands to send over the CAN bus to set the
        /// current face pose.
        ///
        /// \param msgs A vector of messages to send. Only pushes back new
        ///             messages.
        /// \param id   The CAN device ID (default: 253).
        void generateMessages(std::vector<LaboriusMessage>& msgs, int id = 253);

    private:
        uint16_t convertAngle(float v);
        void msgSetData(LaboriusMessage& msg, int motor, float v);

	};

}

#endif
