#ifndef JN0_ARM_COMMON_H
#define JN0_ARM_COMMON_H


#include <dynamic_reconfigure/server.h>
#include <jn0_arm_common/MassPropertiesConfig.h>
#include <eigen3/Eigen/Dense>
#include <hardware_interface/joint_state_interface.h>
#include <tf/tf.h>
#include <ros/ros.h>

//#define NUM_JOINTS 4
//#define NUM_LINKS 5
#define PI 3.14159265
#define GRAVITY 9.81

const std::string NameSpaces[2] = {"jn0_left_arm", "jn0_right_arm"};

// use tf datatypes for Ros Groovy support.
class Jn0ArmCommon
{
public:
  enum ArmSide { LEFT = 0, RIGHT = 1};
  enum {NUM_JOINTS = 4, NUM_LINKS = 5};

  typedef std::vector<hardware_interface::JointStateHandle> JointHandles;

  struct JointsPosition {
    double theta1;
    double theta2;
    double theta3;
    double theta4;
    JointsPosition(double theta1 = 0.0, double theta2 = 0.0, double theta3 = 0.0, double theta4 = 0.0) :
      theta1(theta1), theta2(theta2), theta3(theta3), theta4(theta4)
    {
    }
    double& operator[](unsigned int i)
    {
      assert(i<4);
      switch (i)
      {
        case 0 : return theta1;
        case 1 : return theta2;
        case 2 : return theta3;
        case 3 : return theta4;
      }
      std::cout << "There is a problem in the operator[] overload of the struct JointsPosition, please check it" << std::endl;
      return theta1;
    }

    double get(unsigned int i)
    {
      assert(i<5);
      switch (i)
      {
        case 0 : return theta1;
        case 1 : return theta2;
        case 2 : return theta3;
        case 3 : return theta4;
        case 4 : return 0.0; // it simplify the case of the tool, with this we can make a for up to the tool and don't make a special case for it
      }
      std::cout << "There is a problem in the function get() the struct JointsPosition, please check it" << std::endl;
      return 0.0;
    }

  };

  Jn0ArmCommon(ArmSide as);
//  Jn0ArmCommon(std::string arm_selected);
  ~Jn0ArmCommon();

  void init();

  const std::vector<std::string>& getJointNames() const
  {
      return joint_names_;
  }
  bool PointAt(double x, double y, double z, double& pan, double& tilt, bool constraintAware = false);
  bool PointAt(tf::Vector3 goal, double& pan, double& tilt, bool constraintAware = false);
  // TODO : add other function for other solutions or the closest one

//  btTransform::btTransform GetTransform(std::vector<double> angles, unsigned int begin = 0, unsigned int end = 5);
  tf::Transform GetTransform(JointsPosition angles, unsigned int begin = 0, unsigned int end = 5);

  tf::Vector3 transformVectorFromTo(JointsPosition angles, tf::Vector3 vector, unsigned int from = 0, unsigned int to = 5);
  tf::Vector3 rotateVectorFromTo(JointsPosition angles, tf::Vector3 vector, unsigned int from = 0, unsigned int to = 5);

// TODO: make without torque1, torque2, ...
//  void ComputeGravityTorque(std::vector<double> angles, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass = 0.0);
  void ComputeGravityTorque(JointsPosition angles, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass = 0.0);

  void ComputeGravityTorque(const JointHandles& joints, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass = 0.0);

  bool SimpleIK(double x, double y, double z, double& pan, double& tilt, double& elbow);

  std::string RootName()
  {
    return root_name_;
  }

  std::string TipName()
  {
    return tip_name_;
  }

  std::string GetNameJoint(unsigned int number)
  {
    if(number<NUM_JOINTS)
    {
      return joint_names_[number];
    } else {
      ROS_ERROR("Wrong value  as input in the function Jn0ArmCommon::GetNameJoint");
      return "";
    }
  }

  struct vector6D {
    tf::Vector3 rotational;
    tf::Vector3 linear;
    vector6D() :
      rotational(0,0,0), linear(0,0,0)
    {
    }
    tf::Vector3& operator[](unsigned int i)
    {
      assert(i<2);
      switch (i)
      {
        case 0 : return rotational;
        case 1 : return linear;
      }
      std::cout << "There is a problem in the operator[] overload of the struct vector6D, please check it" << std::endl;
      return rotational;
    }

    double& getElement(unsigned int i)
    {
      assert(i<6);
      switch (i)
      {
        case 0 : return rotational.m_floats[0];
        case 1 : return rotational.m_floats[1];
        case 2 : return rotational.m_floats[2];
        case 3 : return linear.m_floats[0];
        case 4 : return linear.m_floats[1];
        case 5 : return linear.m_floats[2];
      }
      std::cout << "There is a problem in the function get() the struct JointsPosition, please check it" << std::endl;
      return rotational.m_floats[0];
    }
  };

  struct Jacobian {
    vector6D column[NUM_JOINTS];
    vector6D& operator[](unsigned int i)
    {
      assert(i<NUM_JOINTS);
      return column[i];
    }
  };
  Jacobian computeJacobian(JointsPosition angles);

  vector6D computeSpatialVelocity(JointsPosition angles, JointsPosition jointRates);
  // TODO : add gets and sets

  JointsPosition computeTorque(JointsPosition angles, vector6D forces);

  JointsPosition computeCartImpedance(JointsPosition angles, JointsPosition joint_rates, tf::Vector3 pose_goal, tf::Vector3 speed_goal, tf::Vector3 K, tf::Vector3 B, double objectMass = 0.0, tf::Vector3 force_goal = tf::Vector3(0,0,0));
  JointsPosition computeCartImpedance(const JointHandles& joints, tf::Vector3 pose_goal, tf::Vector3 speed_goal, tf::Vector3 K, tf::Vector3 B, double objectMass = 0.0, tf::Vector3 force_goal= tf::Vector3(0,0,0));

  tf::Matrix3x3 skewSymetricMatrix(tf::Vector3 vect);

  // Helper functions

  JointsPosition RobotStateToJointsPosition(const JointHandles& joints)
  {
    return JointsPosition(joints[0].getPosition(),
                          joints[1].getPosition(),
                          joints[2].getPosition(),
                          joints[3].getPosition());
  }

  Eigen::MatrixXd jacobian2MatrixXd(Jacobian jac);

  Eigen::MatrixXd computeInverseJacobian(JointsPosition angles);

  Eigen::VectorXd computeAppliedForce(JointsPosition angles,
                                      double torque1,
                                      double torque2,
                                      double torque3,
                                      double torque4);

  // Not sure that is the right place for this
  // The base of the function has been taken there :
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
  template<typename _Matrix_Type_>
  bool pinv(const _Matrix_Type_ &a, _Matrix_Type_ &result, double
  epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
  {
    if(a.rows()<a.cols())
        return false;

    Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);

    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(),
        a.rows()) * svd.singularValues()[0];

    result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
        tolerance).select(svd.singularValues().
        array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();

    return true;
  }

// Simplest version of the pseudo-inverse
//  void pinv(const Eigen::MatrixXd& mat, Eigen::MatrixXd& inv)
//  {
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
//
//    double epsilon = std::numeric_limits<double>::epsilon();
//
//    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs().matrix().maxCoeff();
//
//    inv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).matrix().select(svd.singularValues().array().inverse().matrix(), 0).asDiagonal() * svd.matrixU().adjoint();
//  }


  /// \brief Set the gravity vector.
  void SetGravity(const tf::Vector3& g)
  {
      gravity_ = g;
  }

private:

  typedef jn0_arm_common::MassPropertiesConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  boost::recursive_mutex dc_mutex_;

//  const std::string NameSpaces[2] = {"jn0_left_arm", "jn0_right_arm"};
  tf::Transform GetTheTransformMatrixI(unsigned int i, double theta);

  void dynamicReconfigureCallback(Config &config, uint32_t level);


  ros::NodeHandle nh_;
  int as_;

  std::string root_name_;
  std::string tip_name_;
  std::vector<std::string> joint_names_;

  double limit_applied_force_;

  bool has_position_limits_[NUM_JOINTS];
  double joint_limit_min_[NUM_JOINTS];
  double joint_limit_max_[NUM_JOINTS];

  double dh_a_[NUM_JOINTS];
  double dh_d_[NUM_JOINTS];
  double dh_alpha_[NUM_JOINTS]; // denavit hartenberg alpha(i-1)
  tf::Vector3 G_[NUM_LINKS];
  double m_[NUM_LINKS];

  double tool_dx_;
  double tool_dy_;
  double tool_dz_;

  tf::Vector3 translation_[5];

  // FF: Measured gravity orientation vector.
  // Initialized as (0, 0, -GRAVITY), will stay as-is if no IMU data is
  // available.
  tf::Vector3 gravity_;

};

#endif //JN0_ARM_COMMON_H
