#include <ros/ros.h>
#include <jn0_face_msgs/FaceExpression.h>
#include <jn0_face_msgs/FacePose.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

namespace jn0_face_expression
{
    /// \brief Normalized period ([0..2pi[)
    static const double NORMALIZED_PERIOD = 2.0 * 3.14159;
    /// \brief Returns a random number between [0, 1.0].
    double drand()
    {
        return double(rand()) / double(RAND_MAX);
    }

    /// \brief Animates Johnny-0's face based on a given face expression.
    ///
    /// This node generates both a face pose (eyebrows and mouth) and PTU bias
    /// that drifts over time.
    /// The drift motion is based on a cosine for each element.
    /// All cosines are based on the same period, but their phases are
    /// randomized.
    /// Period is defined with the input expression.
    /// However, if the defined period is 0, the node reverts to the default
    /// period defined in its parameters.
    /// Amplitude is also randomized, but will always fit between the given
    /// limits (drift value in FaceExpression messages).
    ///
    /// Parameters:
    ///  - drift_period: Default cosine period length in seconds.
    ///    Default: 10 s.
    ///  - period: Update period between sent commands.
    ///    Default: 0.2 s (5 Hz).
    /// Input topics:
    ///  - face_expression: Set the current desired expression.
    ///
    /// Output topics:
    ///  - cmd_face: Final FacePose (with eye angles ignored) meant for the
    ///    face controller.
    ///  - head_bias_pan, head_bias_tilt: Bias angles meant for the PTU
    ///    controller.
    ///
    class ExpressionGenerator
    {
    public:
        ExpressionGenerator()
        {
            ros::NodeHandle np("~");

            np.param("drift_period", default_drift_period_, 10.0);
            drift_period_ = default_drift_period_;
            drift_period_ /= NORMALIZED_PERIOD;

            double period;
            np.param("period", period, 0.2);

            sub_face_exp_ = n_.subscribe("face_expression", 1,
                &ExpressionGenerator::faceCB, this);
            pub_cmd_face_ = 
                n_.advertise<jn0_face_msgs::FacePose>("cmd_face", 1);
            pub_bias_pan_ = 
                n_.advertise<std_msgs::Float64>("head_bias_pan", 1);
            pub_bias_tilt_ = 
                n_.advertise<std_msgs::Float64>("head_bias_tilt", 1);
            
            timer_ = n_.createTimer(ros::Duration(period), 
                &ExpressionGenerator::timerCB, this);

            last_update_ = ros::Time::now();

            // Randomize phase of each cosine.
            srand(last_update_.toNSec());
            for (int i = 0; i < 8; ++i)
                time_[i] = drand();

        }

    private:
        void faceCB(const jn0_face_msgs::FaceExpression::ConstPtr& msg)
        {
            target_ = *msg;
            // Can't use isZero for this, it's (wrongly) non-const:
            if (msg->period.sec == 0 && msg->period.nsec == 0)
                drift_period_ = default_drift_period_;
            else
                drift_period_ = msg->period.toSec();
            drift_period_ /= NORMALIZED_PERIOD;
        }

        void timerCB(const ros::TimerEvent&)
        {
            ros::Time now = ros::Time::now();

            // 1. Update normalized time of each cosine based on the clock.
            // 2. Generate the current command based on the target parameters.
            //    Each value is the mean pose added with the cosine as drift.
            double dt = (now - last_update_).toSec() / drift_period_;
            double cmd[8];
            for (int i = 0; i < 8; ++i)
            {
                time_[i] += dt;
                while (time_[i] >= NORMALIZED_PERIOD)
                    time_[i] -= NORMALIZED_PERIOD;
                cmd[i] = target_.pose[i] + 
                    target_.drift[i] * cos(time_[i]);
            }

            // 3. Generate output messages.
            jn0_face_msgs::FacePose pose;
            pose.left_brow = cmd[0];
            pose.right_brow = cmd[1];
            pose.top_left_mouth = cmd[2];
            pose.top_right_mouth = cmd[3];
            pose.bottom_left_mouth = cmd[4];
            pose.bottom_right_mouth = cmd[5];
            pub_cmd_face_.publish(pose);

            std_msgs::Float64 bias;
            bias.data = cmd[6];
            pub_bias_pan_.publish(bias);
            bias.data = cmd[7];
            pub_bias_tilt_.publish(bias);

            last_update_ = now;

        }

        ros::NodeHandle n_;
        ros::Subscriber sub_face_exp_;
        ros::Publisher pub_cmd_face_;
        ros::Publisher pub_bias_pan_;
        ros::Publisher pub_bias_tilt_;
        ros::Timer timer_;

        jn0_face_msgs::FaceExpression target_;
        ros::Time last_update_;
        double drift_period_;
        double default_drift_period_;
        double time_[8]; // Normalized time ([0.0, 1.0[).

    };
}

int main(int argc, char** argv)
{
    using namespace jn0_face_expression;

    ros::init(argc, argv, "expression_generator");

    ExpressionGenerator n;
    ros::spin();
    
    return 0;
}

