#include <jn0_arm_common/jn0_arm_common.hpp>

// TODO: It should be better to have a function transform which takes a vector to make the multiplication in another direction to win computational efficiency
//       It's computationally better to make T(T(TV)) than (TTT)V (where T is a matrix of transformation and V is a vector)

Jn0ArmCommon::Jn0ArmCommon(Jn0ArmCommon::ArmSide as) : as_(as)
{
  init();
}

Jn0ArmCommon::~Jn0ArmCommon()
{
}

// use tf datatypes for Ros Groovy support.
void Jn0ArmCommon::init()
{
  nh_ = ros::NodeHandle(NameSpaces[as_]);
  assert(nh_.hasParam("joints")); // How verify if there is all the information ? (an idea an bool well_init but like that we can't know where the problem is..)

  nh_.param<std::string>("root_name",root_name_,"no_name");
  nh_.param<std::string>("tip_name",tip_name_,"no_name");

  nh_.param<double>("limit_applied_force", limit_applied_force_, 40);

  ROS_DEBUG_STREAM("Root name = " << root_name_);
  ROS_DEBUG_STREAM("Tip name = " << tip_name_);
  ROS_DEBUG_STREAM("limit_applied_force = " << limit_applied_force_);

  for(int i=0; i<NUM_JOINTS ;i++)
  {
    char tmp_convert_value[2]; // Why two elements ?
    sprintf(tmp_convert_value,"%d",i);


    std::string tmp_joint_selected;
    tmp_joint_selected = "joints/joint_";
    tmp_joint_selected += tmp_convert_value;

    nh_.param<std::string>(tmp_joint_selected + "/name",joint_names_[i],"no_name");
//    nh_.param<uint8_t>(tmp_joint_selected + "/has_pos_limit",info_.limits[i].has_position_limits,false);
//    nh_.getParam(tmp_joint_selected + "/has_pos_limit",info_.limits[i].has_position_limits);
    has_position_limits_[i] = true;
    nh_.param<double>(tmp_joint_selected + "/min_pos",joint_limit_min_[i],0.0);
    nh_.param<double>(tmp_joint_selected + "/max_pos",joint_limit_max_[i],0.0);
    nh_.param<double>(tmp_joint_selected + "/dh_a",dh_a_[i],0.0);
    nh_.param<double>(tmp_joint_selected + "/dh_d",dh_d_[i],0.0);
    nh_.param<double>(tmp_joint_selected + "/dh_alpha",dh_alpha_[i],0.0);

    translation_[i].setX(dh_a_[i]);
    translation_[i].setY(-sin(dh_alpha_[i])*dh_d_[i]);
    translation_[i].setZ(cos(dh_alpha_[i])*dh_d_[i]);

    ROS_DEBUG_STREAM("Joint name = " << joint_names_[i]);
    ROS_DEBUG_STREAM("Has limit = " << has_position_limits_[i]);
    ROS_DEBUG_STREAM("Min pos = " << joint_limit_min_[i]);
    ROS_DEBUG_STREAM("Max pos = " << joint_limit_max_[i]);
    ROS_DEBUG_STREAM("a value of the denavit hartenberg = " << dh_a_[i]);
    ROS_DEBUG_STREAM("d value of the denavit hartenberg = " << dh_d_[i]);
    ROS_DEBUG_STREAM("aplha(i-1) value of the denavit hartenberg = " << dh_alpha_[i]);
  }

  nh_.param<double>("tool_pos/dx",tool_dx_, 0.0);
  nh_.param<double>("tool_pos/dy",tool_dy_, 0.0);
  nh_.param<double>("tool_pos/dz",tool_dz_, 0.0);

  ROS_DEBUG_STREAM("Tool dx = " << tool_dx_);
  ROS_DEBUG_STREAM("Tool dy = " << tool_dy_);
  ROS_DEBUG_STREAM("Tool dz = " << tool_dz_);

  translation_[4].setX(tool_dx_);
  translation_[4].setY(tool_dy_);
  translation_[4].setZ(tool_dz_);


  for(int i=0; i<NUM_LINKS ;i++)
  {
    char tmp_convert_value[2]; // Why two elements ?
    sprintf(tmp_convert_value,"%d",i);


    std::string tmp_link_selected;
    tmp_link_selected = "links/link_";
    tmp_link_selected += tmp_convert_value;

    double tmp;
    nh_.param<double>(tmp_link_selected + "/gc_x",tmp,0.0);
    G_[i].setX(tmp);
    nh_.param<double>(tmp_link_selected + "/gc_y",tmp,0.0);
    G_[i].setY(tmp);
    nh_.param<double>(tmp_link_selected + "/gc_z",tmp,0.0);
    G_[i].setZ(tmp);

    nh_.param<double>(tmp_link_selected + "/mass",m_[i],0.0);

    ROS_DEBUG_STREAM("Gx[" << i << "] = " << G_[i].getX());
    ROS_DEBUG_STREAM("Gy[" << i << "] = " << G_[i].getY());
    ROS_DEBUG_STREAM("Gz[" << i << "] = " << G_[i].getZ());

    ROS_DEBUG_STREAM("mass[" << i << "] = " << m_[i]);

  }

  // TODO find information from the urdf ?


  // The mutex is to avoid a warning when using updateConfig(...).
  reconfigure_server_.reset(new ReconfigureServer(dc_mutex_, nh_));

  // Copy loaded parameters into the server.
  // NOTE: This would automatically if the arm's parameters where loaded in the
  // same namespace.
  Config c;
  c.gc_x_0 = G_[0].x();
  c.gc_y_0 = G_[0].y();
  c.gc_z_0 = G_[0].z();
  c.mass_0 = m_[0];
  c.gc_x_1 = G_[1].x();
  c.gc_y_1 = G_[1].y();
  c.gc_z_1 = G_[1].z();
  c.mass_1 = m_[1];
  c.gc_x_2 = G_[2].x();
  c.gc_y_2 = G_[2].y();
  c.gc_z_2 = G_[2].z();
  c.mass_2 = m_[2];
  c.gc_x_3 = G_[3].x();
  c.gc_y_3 = G_[3].y();
  c.gc_z_3 = G_[3].z();
  c.mass_3 = m_[3];
  c.gc_x_4 = G_[4].x();
  c.gc_y_4 = G_[4].y();
  c.gc_z_4 = G_[4].z();
  c.mass_4 = m_[4];

  reconfigure_server_->updateConfig(c);

  ReconfigureServer::CallbackType f =
                  boost::bind(&Jn0ArmCommon::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // FF: Initial gravity vector, will get periodically updated by the IMU 
  // callback.
  gravity_ = tf::Vector3(0, 0, -GRAVITY);
  
}

tf::Transform Jn0ArmCommon::GetTheTransformMatrixI(unsigned int i, double theta)
{
  assert(i<5);
  tf::Matrix3x3 tmp_mat;
  tmp_mat.setIdentity();
  // use alpha i-1 !
  switch (i){
    case 0 :    tmp_mat.setValue(cos(theta),   -sin(theta),   0.0,
                                 sin(theta),   cos(theta),    0.0,
                                 0.0,          0.0,           1.0);
                break;
    case 1 :    tmp_mat.setValue(cos(theta),   -sin(theta),   0.0,
                                 0.0,          0.0,           1.0,
                                 -sin(theta),  -cos(theta),   0.0);
                break;
    case 2 :    tmp_mat.setValue(-cos(theta),  sin(theta),    0.0,
                                 0.0,          0.0,           -1.0,
                                 -sin(theta),  -cos(theta),   0.0);
                break;
    case 3 :    tmp_mat.setValue(-cos(theta),  sin(theta),    0.0,
                                 0.0,          0.0,           -1.0,
                                 -sin(theta),  -cos(theta),   0.0);
                break;
    case 4 :    tmp_mat.setValue(0.0, 0.0, 1.0,
                                 1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0); //In this case theta is not used, but it's convenient to have the same function even for the tool transform
                break;
    default :   std::cout << "There is a problem in the function Jn0ArmCommon::GetATransformMatrix, please check it" << std::endl;
                break;
  }

  tf::Transform tmp_trans(tmp_mat, translation_[i]);
  return tmp_trans;
}

bool Jn0ArmCommon::PointAt(double x, double y, double z, double& pan, double& tilt, bool constraintAware)
{

  // Every point in the cylinder center at (0,0,0) along the z axis and with a radius of D2 can't be pointed
  if (sqrt(pow(x, 2) + pow(y, 2)) > dh_d_[1])
  {
    // First solution
    pan = acos(dh_d_[1] / sqrt(pow(x, 2) + pow(y, 2))) - atan2(x, y);
    tilt = -atan2(z, sqrt(pow(x, 2) + pow(y, 2) - pow(dh_d_[1], 2))) - PI / 2;

    if((pan < joint_limit_max_[0] and pan > joint_limit_min_[0] and
       tilt < joint_limit_max_[1] and tilt > joint_limit_min_[1]) or !constraintAware)
    {
      return true;
    }

    // Second solution
    pan = -acos(dh_d_[1] / sqrt(pow(x, 2) + pow(y, 2))) - atan2(x, y);
    tilt = -(-atan2(z, sqrt(pow(x, 2) + pow(y, 2) - pow(dh_d_[1], 2))) - PI / 2);

    if((pan < joint_limit_max_[0] and pan > joint_limit_min_[0] and
           tilt < joint_limit_max_[1] and tilt > joint_limit_min_[1]) or !constraintAware)
        {
          return true;
        }

    // No solution
    pan = 0.0;
    tilt = 0.0;

    return false;
  }
  else
  {
    pan = 0.0;
    tilt = 0.0;
    return false;
  }

  return true;
}

bool Jn0ArmCommon::SimpleIK(double x, double y, double z, double& pan, double& tilt, double& elbow)
{
  double tilt_tmp;

  tf::Transform transform02;
  tf::Vector3 point(x,y,z);

  if(PointAt(x,y,z,pan,tilt_tmp,true))
  {

    // Compute l
    JointsPosition angles(pan,0,0,0);
    transform02 = GetTransform(angles,2,0);
    tf::Vector3 point_tmp = transform02*point;

    double l = point_tmp.length();

    // Compute elbow
    double theta4 = PI - acos((pow(tool_dy_,2)+pow(dh_d_[2],2)-pow(l,2))/(2*tool_dy_*-dh_d_[2]));

    ROS_ERROR_STREAM("theta4 " << theta4);
    // Compute tilt
    double theta2_tmp = acos((pow(l,2)+pow(dh_d_[2],2)-pow(tool_dy_,2))/(2*l*-dh_d_[2])); // TODO: verify that d3 = dh_d_[2]
    ROS_ERROR_STREAM("theta2_tmp " << theta2_tmp);

    double theta2 = tilt_tmp + theta2_tmp;

    if (theta2 < joint_limit_max_[1] and theta2 > joint_limit_min_[1] and -theta4 > joint_limit_min_[3]) // Only one check need to be done no ?
    {
      tilt = theta2;
      elbow = -theta4;
      return true;
    } else {
      theta2 = tilt_tmp - theta2_tmp;
      if (theta2 < joint_limit_max_[1] and theta2 > joint_limit_min_[1] and theta4 < joint_limit_max_[3]) // Only one check need to be done no ?
      {
        tilt = theta2;
        elbow = theta4;
        return true;
      } else {
        pan = tilt = elbow = 0.0;
        ROS_ERROR_STREAM("NO in range values");
        return false;
      }
    }

    ROS_ERROR_STREAM("PointAt Fail");
    return false;
  }

  pan = tilt = elbow = 0.0;
  return false;
}

bool Jn0ArmCommon::PointAt(tf::Vector3 goal, double& pan, double& tilt, bool constraintAware)
{
  return PointAt(goal.getX(), goal.getY(), goal.getZ(), pan, tilt, constraintAware);
}

//btTransform::btTransform Jn0ArmCommon::GetTransform(std::vector<double> angles, unsigned int begin, unsigned int end)
//{
//  assert(angles.size()==std::min(std::max(begin,end)-std::min(begin,end),NUM_JOINTS));
//
//  JointsPosition tmp_angles;
//
//
//
//  return Jn0ArmCommon::GetTransform(tmp_angles, begin, end);
//}

tf::Transform Jn0ArmCommon::GetTransform(JointsPosition angles, unsigned int begin, unsigned int end)
{
  assert(begin<=5);
  assert(end<=5);
//  assert(begin!=end);
  bool inverse = !(begin<end);

  tf::Transform tmp_transform;
  tmp_transform.setIdentity();
  for(int i=(int)std::min(begin,end); i<(int)std::max(begin,end); i++)
  {
    tmp_transform *= GetTheTransformMatrixI(i,angles.get(i));
  }

  (inverse)? tmp_transform = tmp_transform.inverse(): tmp_transform;

  return tmp_transform;
}

// This function allows to pass a vector from a reference frame (number 'from') to a new reference (number 'to')
// It can perform both ways
tf::Vector3 Jn0ArmCommon::transformVectorFromTo(JointsPosition angles, tf::Vector3 vector, unsigned int from, unsigned int to)
{
  // TODO: Test this
  assert(from<=5);
  assert(to<=5);

  tf::Transform transformMatrix = GetTransform(angles, to, from);

  return transformMatrix*vector;
}

// This function allows to change the rotation of a vector from a reference frame (number 'from') to a new reference (number 'to')
// It can perform both ways
tf::Vector3 Jn0ArmCommon::rotateVectorFromTo(JointsPosition angles, tf::Vector3 vector, unsigned int from, unsigned int to)
{
  assert(from<=5);
  assert(to<=5);

  tf::Transform transformMatrix = GetTransform(angles, to, from);

  return transformMatrix.getBasis()* vector;
}

//void Jn0ArmCommon::ComputeGravityTorque(std::vector<double> angles, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass)
//{
//  //TODO (vector of double is perhaps better)
//}

void Jn0ArmCommon::ComputeGravityTorque(JointsPosition angles, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass)
{
  tf::Vector3 M[NUM_JOINTS];
  m_[4] += objectMass;

  for(int i=0; i<NUM_JOINTS; i++)
  {
    M[i].setZero();
    tf::Transform tmp_transform;

    // compute gi
    tmp_transform = GetTransform(angles,i+1,0);

//    btVector3::btVector3 tmpgi = gi;
//    gi.setX(tmp_transform.getBasis().getRow(0)[0]*tmpgi.getX()+tmp_transform.getBasis().getRow(0)[1]*tmpgi.getY()+tmp_transform.getBasis().getRow(0)[2]*tmpgi.getZ());
//    gi.setY(tmp_transform.getBasis().getRow(1)[0]*tmpgi.getX()+tmp_transform.getBasis().getRow(1)[1]*tmpgi.getY()+tmp_transform.getBasis().getRow(1)[2]*tmpgi.getZ());
//    gi.setZ(tmp_transform.getBasis().getRow(2)[0]*tmpgi.getX()+tmp_transform.getBasis().getRow(2)[1]*tmpgi.getY()+tmp_transform.getBasis().getRow(2)[2]*tmpgi.getZ());

     // FF: Original transform:
     // gi.setX(-GRAVITY*tmp_transform.getBasis().getRow(0).getZ());
     // gi.setY(-GRAVITY*tmp_transform.getBasis().getRow(1).getZ());
     // gi.setZ(-GRAVITY*tmp_transform.getBasis().getRow(2).getZ());
     tf::Vector3 gi = tmp_transform.getBasis() * gravity_;

     for(int j=i; j<5; j++)
     {
       tmp_transform.setIdentity();
       // compute rji
       tf::Vector3 rji;

       tmp_transform = GetTransform(angles,i+1,j+1);
       rji = tmp_transform*G_[j];

       // compute Mi
       M[i] += m_[j]*(rji.cross(gi));

     }

//     std::cout << "Torque applied on the joint " << i << " = " <<  M[i].getZ() << std::endl;
  }

  m_[4] -= objectMass;

  // get joint torque
  // inverted for compensation
  torque1 =  - M[0].getZ();
  torque2 =  - M[1].getZ();
  torque3 =  - M[2].getZ();
  torque4 =  - M[3].getZ();
}

void Jn0ArmCommon::ComputeGravityTorque(const JointHandles& joints, double& torque1, double& torque2, double& torque3, double& torque4, double objectMass)
{
  Jn0ArmCommon::JointsPosition angles(joints[0].getPosition(),
                                      joints[1].getPosition(),
                                      joints[2].getPosition(),
                                      joints[3].getPosition());

  ComputeGravityTorque(angles,torque1,torque2,torque3,torque4,objectMass);
}

void Jn0ArmCommon::dynamicReconfigureCallback(Config &config, uint32_t level)
{
  ROS_DEBUG("Reconfigure Request: %f %f %f %f",
            config.gc_x_0, config.gc_y_0,
            config.gc_z_0, config.mass_0);

  G_[0].setX(config.gc_x_0);
  G_[0].setY(config.gc_y_0);
  G_[0].setZ(config.gc_z_0);
  m_[0] = config.mass_0;

  G_[1].setX(config.gc_x_1);
  G_[1].setY(config.gc_y_1);
  G_[1].setZ(config.gc_z_1);
  m_[1] = config.mass_1;

  G_[2].setX(config.gc_x_2);
  G_[2].setY(config.gc_y_2);
  G_[2].setZ(config.gc_z_2);
  m_[2] = config.mass_2;

  G_[3].setX(config.gc_x_3);
  G_[3].setY(config.gc_y_3);
  G_[3].setZ(config.gc_z_3);
  m_[3] = config.mass_3;

  G_[4].setX(config.gc_x_4);
  G_[4].setY(config.gc_y_4);
  G_[4].setZ(config.gc_z_4);
  m_[4] = config.mass_4;

}

tf::Matrix3x3 Jn0ArmCommon::skewSymetricMatrix(tf::Vector3 vect)
{
  return tf::Matrix3x3(     0,       -vect.getZ(),  vect.getY(),
                      vect.getZ(),      0,       -vect.getX(),
                     -vect.getY(),  vect.getX(),      0      );
}

Jn0ArmCommon::Jacobian Jn0ArmCommon::computeJacobian(JointsPosition angles)
{
  Jacobian jacob;
//  std::cout << jacob[0][0].getX() << " " << jacob[0][0].getY() << " " << jacob[0][0].getZ() << " " << jacob[0][1].getX() << " " << jacob[0][1].getY() << " " << jacob[0][1].getZ() << std::endl;
//  std::cout << jacob[1][0].getX() << " " << jacob[1][0].getY() << " " << jacob[1][0].getZ() << " " << jacob[1][1].getX() << " " << jacob[1][1].getY() << " " << jacob[1][1].getZ() << std::endl;
//  std::cout << jacob[2][0].getX() << " " << jacob[2][0].getY() << " " << jacob[2][0].getZ() << " " << jacob[2][1].getX() << " " << jacob[2][1].getY() << " " << jacob[2][1].getZ() << std::endl;
//  std::cout << jacob[3][0].getX() << " " << jacob[3][0].getY() << " " << jacob[3][0].getZ() << " " << jacob[3][1].getX() << " " << jacob[3][1].getY() << " " << jacob[3][1].getZ() << std::endl;

  tf::Vector3 iPi(0,0,0); // Vector p of i expressed in the frame i
  tf::Vector3 iZi(0,0,1); // Vector Z of i expressed in the frame i

  // Loop for to compute the column for every joint
  for(int i=0; i<NUM_JOINTS; i++)
  {
    // Compute pi in the tool frame
    tf::Transform tmp_transform = GetTransform(angles, 5, i+1);
    tf::Vector3 tPi = tmp_transform*iPi; // Vector p of i expressed in the tool frame
    // Compute S(pi)
    tf::Matrix3x3 StPi = skewSymetricMatrix(tPi); // Skew matrix made with the vector tPi
    // Compute Zi in the tool frame
    tf::Vector3 tZi= tmp_transform.getBasis().getColumn(2); // The Z axis of i expressed in the frame t
    // update column i of the Jacobian
    jacob[i][0] = tZi;
    jacob[i][1] = StPi*tZi;
  }

//  std::cout << "Jacobian :" << std::endl;
//  std::cout << jacob[0][0].getX() << "\t" << jacob[0][0].getY() << "\t" << jacob[0][0].getZ() << "\t" << jacob[0][1].getX() << "\t" << jacob[0][1].getY() << "\t" << jacob[0][1].getZ() << std::endl;
//  std::cout << jacob[1][0].getX() << "\t" << jacob[1][0].getY() << "\t" << jacob[1][0].getZ() << "\t" << jacob[1][1].getX() << "\t" << jacob[1][1].getY() << "\t" << jacob[1][1].getZ() << std::endl;
//  std::cout << jacob[2][0].getX() << "\t" << jacob[2][0].getY() << "\t" << jacob[2][0].getZ() << "\t" << jacob[2][1].getX() << "\t" << jacob[2][1].getY() << "\t" << jacob[2][1].getZ() << std::endl;
//  std::cout << jacob[3][0].getX() << "\t" << jacob[3][0].getY() << "\t" << jacob[3][0].getZ() << "\t" << jacob[3][1].getX() << "\t" << jacob[3][1].getY() << "\t" << jacob[3][1].getZ() << std::endl;


  // return the computed Jacobian
  return jacob;
}

Jn0ArmCommon::vector6D Jn0ArmCommon::computeSpatialVelocity(JointsPosition angles, JointsPosition jointRates)
{
  vector6D spatialVelocity;

  // Compute the Jacobian
  Jacobian jacob = computeJacobian(angles);

  for(int j = 0; j < 6; j++)
  {
    double sum = 0;
    for(int i = 0; i < NUM_JOINTS; i++)
    {
      sum += jacob[i].getElement(j)*jointRates[i];
    }

    spatialVelocity.getElement(j) = sum;
  }

  return spatialVelocity;
}

Jn0ArmCommon::JointsPosition Jn0ArmCommon::computeTorque(JointsPosition angles, vector6D forces)
{
  JointsPosition torque;

  // Compute the Jacobian
  Jacobian jacob = computeJacobian(angles);

  for(int j = 0; j < NUM_JOINTS; j++)
  {
    double sum = 0;
    for(int i = 3; i < 6; i++)
    {
      sum += jacob[j].getElement(i)*forces.getElement(i);
    }

    torque[j] = sum;
  }

  return torque;
}

Jn0ArmCommon::JointsPosition Jn0ArmCommon::computeCartImpedance(JointsPosition angles, JointsPosition joint_rates, tf::Vector3 pose_goal, tf::Vector3 speed_goal, tf::Vector3 K, tf::Vector3 B, double objectMass, tf::Vector3 force_goal)
{
  // This function computes the torque for every joint, the formula implemented is :
  // Torques = transpose_jacobian(angles)*(stiffness_parameter_K*(pose_goal-direct_kinematic(angles)) + damping_parameter_B*(speed_goal-jacobian(angles)*joint_rates))

  // TODO: Optimise code
  vector6D impedance_forces;

  tf::Transform transform = GetTransform(angles, 0, 5);
  tf::Vector3 pose_actual = transform.getOrigin();

  tf::Vector3 stiffness_part = K*(pose_goal-pose_actual); // elementwise multiplication

  vector6D speed_actual_6D = computeSpatialVelocity(angles,joint_rates);
  tf::Vector3 speed_actual = speed_actual_6D.linear;

  tf::Vector3 damping_part = B*(speed_goal-speed_actual); // elementwise multiplication

  impedance_forces.rotational = tf::Vector3(0.0,0.0,0.0);

  //impedance_forces.linear = stiffness_part + damping_part + transform.inverse().getBasis()*force_goal;
  impedance_forces.linear = transform.inverse().getBasis()*(stiffness_part + damping_part + force_goal);


  double force_norm = impedance_forces.linear.length();

  if(force_norm>limit_applied_force_)
  {
    impedance_forces.linear *= (limit_applied_force_/force_norm);
  }

  double torque1, torque2, torque3, torque4 = 0.0;

  ComputeGravityTorque(angles, torque1, torque2, torque3, torque4, objectMass);

  JointsPosition impedance_torque = computeTorque(angles, impedance_forces);

  impedance_torque.theta1 += torque1;
  impedance_torque.theta2 += torque2;
  impedance_torque.theta3 += torque3;
  impedance_torque.theta4 += torque4;

  return impedance_torque;
}

Jn0ArmCommon::JointsPosition Jn0ArmCommon::computeCartImpedance(const JointHandles& joints, tf::Vector3 pose_goal, tf::Vector3 speed_goal, tf::Vector3 K, tf::Vector3 B, double objectMass, tf::Vector3 force_goal)
{
  Jn0ArmCommon::JointsPosition angles(joints[0].getPosition(),
                                      joints[1].getPosition(),
                                      joints[2].getPosition(),
                                      joints[3].getPosition());

  Jn0ArmCommon::JointsPosition joint_rates(joints[0].getVelocity(),
                                           joints[1].getVelocity(),
                                           joints[2].getVelocity(),
                                           joints[3].getVelocity());

  return computeCartImpedance(angles, joint_rates, pose_goal, speed_goal, K, B, objectMass, force_goal);
}

Eigen::MatrixXd Jn0ArmCommon::jacobian2MatrixXd(Jacobian jac)
{
  Eigen::MatrixXd jac_eig(6,4);

  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 6; j++)
    {
      jac_eig(j,i) = jac[i].getElement(j);
    }
  }

  return jac_eig;
}

Eigen::MatrixXd Jn0ArmCommon::computeInverseJacobian(JointsPosition angles)
{
  Jacobian jac = computeJacobian(angles);

  Eigen::MatrixXd jac_eig = jacobian2MatrixXd(jac);

  Eigen::MatrixXd inv(4, 6);

  pinv(jac_eig, inv);

  return inv;
}

Eigen::VectorXd Jn0ArmCommon::computeAppliedForce(JointsPosition angles,
                                                  double applied_torque1,
                                                  double applied_torque2,
                                                  double applied_torque3,
                                                  double applied_torque4)
{
  Eigen::MatrixXd pinv_jacobian = computeInverseJacobian(angles);

  Eigen::Vector4d applied_torques(applied_torque1,
                                  applied_torque2,
                                  applied_torque3,
                                  applied_torque4);

  return (pinv_jacobian.transpose()*applied_torques);
}

