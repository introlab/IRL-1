#include <ros/ros.h>
#include <jn0_face_msgs/FaceExpression.h>
#include <jn0_face_msgs/EmoPulse.h>
#include <jn0_face_msgs/EmoIntensity.h>
#include <boost/array.hpp>
#include <tr1/unordered_map>
#include <string>
#include <algorithm>
#include <numeric>

namespace jn0_face_expression
{
    /// \brief Pose vector size, also valid for drift.
    static const unsigned int POSE_SIZE = 8;

    /// \brief A node that translates emotion intensities into face expressions
    /// for Johnny-0.
    ///
    /// At startup, this node registers a series of expressions each mapped to
    /// a single emotion (Fear, Anger, Joy, ...).
    /// At each update, the node output a facial expression that linearly
    /// combines the mapped expressions from the output of the emotions
    /// model.
    /// Unknown emotion types are simply ignored.
    ///
    /// Temporarily, an emotional pulse can be added to the output of the
    /// model.
    /// A pulse p starts at a given intensity i and decreases over a given 
    /// time period n, following this:
    ///
    ///   p = i * ((n-t)/n)^2, with t = [0,n]
    /// 
    /// The result of p is added to the emotion type given with the pulse.
    /// A new pulse overwrite whatever was present for its emotion type.
    /// In other words, only one pulse is tracked per emotion, but multiple
    /// pulses (for different emotion types) can be blended.
    /// With this in mind, emotional pulses should have sensible durations.
    /// See the jn0_face_msgs/EmoPulse message type for interface details.
    ///
    ///
    /// Parameters:
    ///  - min_pulse_duration: Minimum pulse duration.
    ///    Shorter pulse durations are set to this value.
    ///    Default: 1.0 s.
    ///  - period: Update period, in seconds.
    ///    Default: 0.1 s (10 Hz).
    ///  - expressions: An array of face expressions mapped to emotion types.
    ///    The format of each mapping in YAML is this:
    ///
    ///    {
    ///      emo_type: string
    ///      expression: { # Same format as jn0_face_msgs/FaceExpression
    ///        pose: [float] 
    ///        mask: [float]
    ///        drift: [float]
    ///        period: float
    ///    } 
    ///   
    ///    The mask field in expressions is used by the linear combinator.
    ///    It uses the component-wise product of the pose and mask as input
    ///    so that expressions can be defined as a subset of the whole face
    ///    pose.
    ///    It also applies to drift calculation in the same manner.
    ///    The mask values should be between 0.0 and 1.0.
    ///    If the mask is not defined, a full vector of ones is used as default.
    ///
    ///    NOTE: This module currently only supports one expression mapping
    ///    per emotion type.
    ///    A second mapping with the same emotion type will simply overwrite
    ///    the previous one.
    ///  
    /// Input topics:
    ///  - emo_intensity: Output of the emotions model.
    ///    Each incoming message, if it can be mapped, alters the internal 
    ///    expression model.
    ///  - emo_intensities: Output of the emotions model (as a vector). 
    ///  - emo_pulse: A temporary emotional pulse input.
    ///    See jn0_face_msgs/EmoPulse for details.
    ///
    /// Output topics:
    ///  - face_expression: The combined expression output.
    ///
    class EmotionTranslator
    {
    public:
        typedef boost::array<double, POSE_SIZE> PoseType;

        EmotionTranslator()
        {
            ros::NodeHandle np("~");

            sub_emo_intensity_ = n_.subscribe("emo_intensity", 1,
                &EmotionTranslator::intensityCB, this);
            sub_emo_intensities_ = n_.subscribe("emo_intensities", 1,
                &EmotionTranslator::intensityCB, this);
            sub_emo_pulse_ = n_.subscribe("emo_pulse", 1,
                &EmotionTranslator::pulseCB, this);

            pub_expr_ = n_.advertise<jn0_face_msgs::FaceExpression>(
                "face_expression", 1);


            generateMappings(np);

            double p;
            np.param("min_pulse_duration", p, 1.0);
            min_pulse_duration_ = ros::Duration(p);
            np.param("period", p, 0.1);
            timer_ = n_.createTimer(ros::Duration(p),
                &EmotionTranslator::timerCB, this);
        }

    private:
        void generateMappings(ros::NodeHandle& np)
        {
            using namespace XmlRpc;

            if (!np.hasParam("expressions"))
            {
                ROS_WARN("No expression mappings given!");
                return;
            }

            XmlRpcValue mappings;
            np.getParam("expressions", mappings);

            if (mappings.getType() != XmlRpcValue::TypeArray)
            {
                ROS_ERROR("Expression mappings parameter isn't an array!");
                return;
            }

            for (int i = 0; i != mappings.size(); ++i)
            {
                XmlRpcValue& e = mappings[i];
                if (e.getType() != XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("Expression mapping isn't a struct!");
                    continue;
                }

                if (!e.hasMember("emo_type"))
                {
                    ROS_ERROR("Undefined emo_type in expression mapping!");
                    continue;
                }

                // Erase any previously saved mapping.
                const std::string& emo_type = e["emo_type"];
                ModelElement& m = model_[emo_type] = ModelElement();

                if (e.hasMember("pose"))
                {
                    parsePoseArray(e["pose"], m.pose);
                }
                else
                    ROS_WARN("No pose given for %s.", emo_type.c_str());

                if (e.hasMember("mask"))
                {
                    parsePoseArray(e["mask"], m.mask);
                }

                if (e.hasMember("drift"))
                {
                    parsePoseArray(e["drift"], m.drift);
                }
                else
                    ROS_WARN("No drift given for %s.", emo_type.c_str());

                if (e.hasMember("period"))
                {
                    XmlRpcValue& p = e["period"];
                    if (p.getType() != XmlRpcValue::TypeDouble)
                    {
                        ROS_WARN("Wrong period format for %s!", 
                            emo_type.c_str());
                    }
                    else
                        m.period = ros::Duration(p);
                }
                    
            }

            
        }

        void parsePoseArray(XmlRpc::XmlRpcValue& pose, PoseType& out)
        {
            using namespace XmlRpc;

            if (pose.getType() != XmlRpcValue::TypeArray)
            {
                ROS_ERROR("Given pose isn't an array!");
                return;
            }

            if (pose.size() != int(POSE_SIZE))
            {
                ROS_ERROR("Pose array isn't of size %d!", POSE_SIZE);
                return;
            }

            for (unsigned int i = 0; i < POSE_SIZE; ++i) 
            {
                XmlRpcValue& p = pose[i];
                if (p.getType() != XmlRpcValue::TypeDouble)
                {
                    ROS_ERROR("Given pose element cannot be parsed as double!");
                    out[i] = 0.0;
                }
                out[i] = p;
            }
        }

        void applyIntensity(const jn0_face_msgs::EmoIntensity& msg)
        {
            const std::string& emo_type = msg.name;
            ModelType::iterator i = model_.find(emo_type);
            if (i == model_.end())
            {
                ROS_ERROR("Unknown emotion type for pulse: %s", 
                    emo_type.c_str());
                return;
            }

            ModelElement& e = i->second;
            e.value = msg.value;
        }

        void intensityCB(const jn0_face_msgs::EmoIntensity::ConstPtr& msg)
        {
            applyIntensity(*msg);
        }

        void pulseCB(const jn0_face_msgs::EmoPulse::ConstPtr& msg)
        {
            ModelType::iterator i = model_.find(msg->emo_type);
            if (i == model_.end())
            {
                ROS_ERROR("Unknown emotion type for pulse: %s", 
                    msg->emo_type.c_str());
                return;
            }

            ModelElement& e = i->second;
            e.pulse_start = ros::Time::now();
            e.pulse_duration = msg->duration;
            e.pulse_intensity = msg->intensity;
            if (e.pulse_duration < min_pulse_duration_)
                e.pulse_duration = min_pulse_duration_;
        }

        void timerCB(const ros::TimerEvent&)
        {
            ros::Time now = ros::Time::now();

            // First pass: cycle through model elements, calculate gains.
            std::vector<double> gains;
            gains.reserve(model_.size());
            for (ModelType::iterator i = model_.begin(); i != model_.end(); ++i)
            {
                const std::string& emo_type = i->first;
                ModelElement& e = i->second;

                // Pulse effect update.
                // See class documentation for details.
                double p = 0.0;
                if (now < (e.pulse_start + e.pulse_duration))
                {
                    double t = (now - e.pulse_start).toSec();
                    const double& intensity = e.pulse_intensity;
                    const double& n = e.pulse_duration.toSec();
                    p = (n - t) / n;
                    p = intensity * p * p;
                }

                double output = p + e.value;
                ROS_DEBUG_THROTTLE(0.5, "Output for %s: %f",
                    emo_type.c_str(), output);
                gains.push_back(output);
            }

            // Check if normalization is needed.
            // We only do it if the sum could go over 1.0.
            // Always normalizing would result in exaggerated expressions for
            // low intensities.
            double gains_sum = 0.0;
            std::accumulate(gains.begin(), gains.end(), gains_sum);
            if (gains_sum > 1.0 && gains_sum > 0.0)
            {
                std::vector<double>::iterator i;
                for (i = gains.begin(); i != gains.end(); ++i)
                    *i /= gains_sum;
            }

            // Second pass: cycle through model elements again, generating
            // the final expression with the calculated gains.
            jn0_face_msgs::FaceExpression out;
            // Might be redundant:
            std::fill(out.pose.begin(), out.pose.end(), 0.0);
            std::fill(out.drift.begin(), out.drift.end(), 0.0);
            double expr_period = 0;
            ModelType::const_iterator i;
            unsigned int j = 0;
            for (i = model_.begin(), j = 0; i != model_.end(); ++i, ++j)
            {
                const double& gain = gains[j];
                const PoseType& pose = i->second.pose;
                const PoseType& mask = i->second.mask;
                const PoseType& drift = i->second.drift;
                double period = i->second.period.toSec();

                for (unsigned int k = 0; k < POSE_SIZE; ++k)
                {
                    out.pose[k] += gain * mask[k] * pose[k];
                    out.drift[k] += gain * mask[k] * drift[k];
                }
                expr_period += gain * period;
            }

            out.period = ros::Duration(expr_period);

            pub_expr_.publish(out);


        }

        ros::NodeHandle n_;
        ros::Subscriber sub_emo_intensity_;
        ros::Subscriber sub_emo_intensities_;
        ros::Subscriber sub_emo_pulse_;
        ros::Publisher pub_expr_;
        ros::Timer timer_;

        ros::Duration min_pulse_duration_;

        /// \brief A single emotional element for the internal model, include
        /// pulse data.
        struct ModelElement
        {
            ModelElement():
                value(0), 
                pulse_start(0), pulse_duration(0), pulse_intensity(0.0), 
                period(0)
            {
                std::fill(pose.begin(), pose.end(), 0.0);
                std::fill(mask.begin(), mask.end(), 1.0);
                std::fill(drift.begin(), drift.end(), 0.0);
            }

            // Current intensity:
            double value;

            // Pulse management: 
            ros::Time pulse_start;
            ros::Duration pulse_duration;
            double pulse_intensity;

            // Single mapped pose output:
            PoseType pose;
            PoseType mask;
            PoseType drift;
            ros::Duration period;
        };

        typedef std::tr1::unordered_map<std::string, ModelElement> ModelType;
        ModelType model_;


    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emotion_translator");

    jn0_face_expression::EmotionTranslator n;
    ros::spin();

    return 0;

}

