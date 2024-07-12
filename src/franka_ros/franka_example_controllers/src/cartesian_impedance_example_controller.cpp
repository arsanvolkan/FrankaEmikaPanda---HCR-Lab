// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <string>

#include <vector>

#include <franka_example_controllers/pseudo_inversion.h>


// need to do: have a function to read the file writen with joint x,y,z,w,wx,wy,wz,t
//use this directly for the target pose
// save function, same format from position and position_d, have a python file to print traj of files of this formats.

// To save traj file (added by Camille)
void Write_Line_To_File(std::string file_name, std::vector<double> dataToSave, ros::Duration time) { // modified : added function to save data in file
  std::ofstream myfile;
  myfile.open(file_name, std::ios_base::app);
  if (!myfile)
  {
    std::cout << "No file found: " << file_name << std::endl;
    return;
  }
  for (int j = 0; j < dataToSave.size(); j++)
  {
    if (j == dataToSave.size() - 1) {
      myfile << dataToSave[j] << " " << time << std::endl;
    }
    else {
      myfile << dataToSave[j] << " ";
    }
  }
  myfile.close();
}

//int loadVectorMatrixFromFile (std::string fileName, int cols, std::vector<std::vector<double>> &outMat)
//{
//  ROS_INFO("ooooooooooooo in loadVectorMatrixFromFile oooooooooooooo");
//  std::ifstream in(fileName.data());
//  if (!in)
//  {
//    std::cout << "No file found: " << fileName << std::endl;
//    return -1;
//  }
//  int counter = 0;
//  while (!in.eof())
//  {
//    outMat.push_back( std::vector <double>() );
//    for (int j = 0; j < cols; ++j)
//    {
//      double readf;
//      in >> readf;
//      outMat[counter].push_back(readf);
//      ROS_INFO("%lf",outMat[counter]);
//    }
//    counter++;
//  }
//  outMat.pop_back();
//  in.close();
//  return 0;
//}


//Cartesian impedance control uses the dynamic model, the joint efforts, the robot state
namespace franka_example_controllers {
//std::vector<std::vector<double>> outMat;
bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector; // Where are they used???????
  std::vector<double> cartesian_damping_vector;

  //subscribe to equilibrium node
  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  //check arm existence
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  //check all joints are defined
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  //std::string file_name;
  //if (!node_handle.getParam("file_name", file_name){
  //  ROS_ERROR(
  //      "CartesianImpedanceExampleController: Invalid or no file_name parameters provided, "
  //      "aborting controller init!");
  //  return false;
  //}

  //get model interface from hw
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }

  //get model handle from hw
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  //get state interface from hw
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }

  //get state handle from hw
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }


  

  //get effort joint interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  //init node dynamic_reconfigure_compliance_param_node_
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  //init pose
  position_d_.setZero(); //all coeff of position_d_ are set to 0
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  //get robot trajectory (added by Camille)

  //ros::param::get("/file_name", file_name2_);
  ROS_INFO("ooooooooooooo in loadVectorMatrixFromFile oooooooooooooo");
  std::ifstream in("/home/dhrikarl/catkin_ws/joint_impedance_position_schoolVisit.txt"); //8 cols
  //std::ifstream in("/home/dhrikarl/catkin_ws/joint_impedance_position_"+ file_name2_ +  ".txt");
  //std::ifstream in("/home/dhrikarl/catkin_ws/planDMP_testdemoforPERSEO_touchFruitb.txt"); //4 cols

  if (!in)
  {
    std::cerr << "Could not open the file" << std::endl;
    return EXIT_FAILURE;
  }
  int counter = 0;
  int cols=8;   //8 if you read x,y,z,w,wx,wy,wz,t or 16 if you read transformation matrix or 4 if you read only the position
  while (!in.eof())
  {
    outMat_.push_back( std::vector <double>() );
    for (int j = 0; j < cols; ++j)
    {
      double readf;
      in >> readf;
      if (!in.eof()) {
        ROS_INFO("readf value is %lf", readf);
        outMat_[counter].push_back(readf);
        //ROS_INFO("%lf", outMat.end());
        //ROS_INFO("j value is %i", j);
      }

    }
    ROS_INFO("counter value is %i",counter);
    counter=counter+1;
  }
  outMat_.pop_back();
  in.close();


  return true;
}


void CartesianImpedanceExampleController::starting(const ros::Time& time) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  int step=0;
  initial_time_=time;
  franka::RobotState initial_state = state_handle_->getRobotState(); //get all state of robot, including O_T_EE ..etc coming from state_interface->getHandle(robot)
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  // Eigen::Map "A matrix or vector expression mapping an existing array of data", in the following example it creates a 1*7 matrix with the elements from q
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data()); //q_initial is the articulation coming from the robot at the initial state
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())); // creates an element of the type Affine3D which is a 4*4 homogeneous transformation matrix from the data in O_T_EE concerning the EE

  // set equilibrium point to current state
  position_d_ = initial_transform.translation(); //extract the translation (x,y,z) from the homogenous transformation matrix
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear()); // converts to a quaternion the 3*3 rotational matrix obtained from the homogeneous transformation matrix
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

}

void CartesianImpedanceExampleController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  if (step < outMat_.size()) {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;
    std::vector<double> v1;
    v1.resize(position.size());
    Eigen::Vector3d::Map(&v1[0], position.size()) = position;
    Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_schoolVisit.txt",v1,time-initial_time_ );
    //Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_"+file_name2_+".txt",v1,time-initial_time_);
    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse643
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // Desired torque
    tau_d << tau_task + 0*tau_nullspace + coriolis;  //modified to remove nullspace
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering  //modified to constant stiffness and damping
  cartesian_stiffness_ =
      2000* Eigen::MatrixXd::Identity(6,6); cartesian_stiffness_(3,3)=100 ;cartesian_stiffness_(4,4)=100 ;cartesian_stiffness_(5,5)=100 ;
  cartesian_damping_ =
      2*0.7*cartesian_stiffness_.sqrt() ;
  //cartesian_stiffness_ =
    //  filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  //cartesian_damping_ =
  //    filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

//  std::lock_guard<std::mutex> position_d_target_mutex_lock(
//      position_and_orientation_d_target_mutex_);


  // Replaces call to mutex because it slows down the process
  std::vector<double> T=outMat_[step];
  std::vector<double> last_transformation_matrix = outMat_.back();
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  position_d_target_ << T[0], T[1], T[2];

  // Update the target orientation if you want to reproduce the whole orientation trajectory:
  orientation_d_target_.w() = T[3];
  orientation_d_target_.x() = T[4];
  orientation_d_target_.y() = T[5];
  orientation_d_target_.z() = T[6];

  // Update the target orientation if you want to generate an orientation trajectory from the first and last orientation using slerp , careful this should be moved outside the while loop because it doesn't change
  //orientation_d_target_.w() = last_transformation_matrix[3];
  //orientation_d_target_.x() = last_transformation_matrix[4];
  //orientation_d_target_.y() = last_transformation_matrix[5];
  //orientation_d_target_.z() = last_transformation_matrix[6];

  // Get the default start orientation 0.00039158, 0.999997, -5.56286e-05, -0.00070721
  //orientation_d_target_.w() = 0.00039158;
  //orientation_d_target_.x() = 0.999997;
  //orientation_d_target_.y() = -0.0000556286;
  //orientation_d_target_.z() = -0.00070721;

  // Get the default final orientation 0.0490364 0.762398 -0.642465 -0.0597939 orange OR 0.0186346 0.888802 -0.445446 -0.106096 banana
  //orientation_d_target_.w() = 0.0186346;
  //orientation_d_target_.x() = 0.888802;
  //orientation_d_target_.y() = -0.445446;
  //orientation_d_target_.z() = -0.106096;


  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  step = step + 1;
  //

  std::vector<double> position_d_target_vector;
  position_d_target_vector.resize(position_d_target_.size());
  Eigen::Vector3d::Map(&position_d_target_vector[0], position_d_target_.size()) = position_d_target_;

  ROS_INFO("x: %lf",position_d_target_vector[0]);
  ROS_INFO("y: %lf",position_d_target_vector[1]);
  ROS_INFO("z: %lf",position_d_target_vector[2]);
  ROS_INFO("w: %lf",orientation_d_target_.w());
  ROS_INFO("wx: %lf",orientation_d_target_.x());
  ROS_INFO("wy: %lf",orientation_d_target_.y());
  ROS_INFO("wz: %lf",orientation_d_target_.z());
  //ROS_INFO("step: %i",step);

  // UPDATE POSITION
  position_d_ = position_d_target_; //modified to follow exactly my trajectory
  //position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_; //filter_param_=0.005 // divide filter_param_ to reduce the velocity?

  // UPDATE ORIENTATION
  orientation_d_ = orientation_d_target_; //modified to follow exactly my trajectory
  //orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);


  //Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_test1.txt",position );
  //Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_d_test1.txt",position_d_ );

  std::vector<double> v2;
  std::vector<double> v_orientation;
  v2.resize(position_d_.size());
  v_orientation.push_back(orientation_d_.w());
  v_orientation.push_back(orientation_d_.x());
  v_orientation.push_back(orientation_d_.y());
  v_orientation.push_back(orientation_d_.z());
  Eigen::Vector3d::Map(&v2[0], position_d_.size()) = position_d_;
  
  Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_d_schoolVisit.txt",v2,time-initial_time_ );
  Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_orientation_d_schoolVisit.txt",v_orientation,time-initial_time_ );
  //Write_Line_To_File("/home/dhrikarl/catkin_ws/cartesian_impedance_position_d_"+file_name2_+".txt",v2,time-initial_time_ );
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceExampleController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}



void CalculateRotation(std::vector<double> &T, std::vector<double> &quaternion)  { //
  double S;
  if ((T[0] > T[5])&&(T[0] > T[10])) {
    S = sqrt( 1.0 + T[0] - T[5] - T[10] ) * 2;
    quaternion[0] = 0.25 / S;
    quaternion[1] = (T[1] + T[4] ) / S;
    quaternion[2] = (T[2] + T[8] ) / S;
    quaternion[3] = (T[6] - T[9] ) / S;

    }
  else if (T[5] > T[10]) {
      S = sqrt( 1.0 + T[5] - T[0] - T[10] ) * 2;
      quaternion[0] = (T[1] + T[4] ) / S;
      quaternion[1] = 0.25 / S;
      quaternion[2] = (T[6] + T[9] ) / S;
      quaternion[3] = (T[2] - T[8] ) / S;

    }
  else {
      S = sqrt( 1.0 + T[10] - T[0] - T[5] ) * 2;
      quaternion[0] = (T[2] + T[8] ) / S;
      quaternion[1] = (T[6] + T[9] ) / S;
      quaternion[2] = 0.25 / S;
      quaternion[3] = (T[1] - T[4] ) / S;
  }
}

// NOT USED CURRENTLY
void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // Modified: this functions is where the target values are given to the controller
  // it has been modified to use either values coming from a file of the translations and rotations
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);

  int cols=8;   //8 if you read x,y,z,w,wx,wy,wz,t or 16 if you read transformation matrix

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  if (cols == 16){
    std::vector<double> quaternion={0,0,0,0};
    std::vector<double> T=outMat_[step];
    CalculateRotation(T,quaternion);
    position_d_target_ << T[12], T[13], T[14];
    orientation_d_target_.coeffs() << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
  }

  if (cols == 8){
    std::vector<double> T=outMat_[step];
    position_d_target_ << T[0], T[1], T[2];
    //orientation_d_target_.coeffs() << T[3], T[4], T[5], T[6];
    orientation_d_target_.w() = T[3];
    orientation_d_target_.x() = T[4];
    orientation_d_target_.y() = T[5];
    orientation_d_target_.z() = T[6];
  }

  if (cols == 4){
    std::vector<double> quaternion={0.00039158, 0.999997, -5.56286e-05, -0.00070721};
    std::vector<double> T=outMat_[step];
    position_d_target_ << T[0], T[1], T[2];
    orientation_d_target_.coeffs() << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
  }

  //step=step+1;
  std::vector<double> position_d_target_vector1;
  position_d_target_vector1.resize(position_d_target_.size());
  Eigen::Vector3d::Map(&position_d_target_vector1[0], position_d_target_.size()) = position_d_target_;
  ROS_INFO("x1: %lf",position_d_target_vector1[0]);
  ROS_INFO("y1: %lf",position_d_target_vector1[1]);
  ROS_INFO("z1: %lf",position_d_target_vector1[2]);
  ROS_INFO("w1: %lf",orientation_d_target_.w());
  ROS_INFO("wx1: %lf",orientation_d_target_.x());
  ROS_INFO("wy1: %lf",orientation_d_target_.y());
  ROS_INFO("wz1: %lf",orientation_d_target_.z());
  ROS_INFO("step1: %i",step);
  //if you want to read the values from RVIZ, and go back to the initial example from Franka
  //you should comment if (cols==16) and if (cols==8) and uncomment the following lines:
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(
    //  position_and_orientation_d_target_mutex_);
  //position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  //Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  //orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //    msg->pose.orientation.z, msg->pose.orientation.w;

  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
