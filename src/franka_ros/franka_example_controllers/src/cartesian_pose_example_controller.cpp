// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <string.h>
#include <math.h>
#include <array>
#include <algorithm>
#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
using namespace std;
using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::vector;

namespace franka_example_controllers {
    //addedd variables start
    vector<string> trackerLines;
    std::vector<std::array <double,3>> trackerVectors;
    std::array <double,3> orgTrackerPose;
    vector<double> trackerArrayX;
    vector<double> trackerArrayY;
    vector<double> trackerArrayZ;
    string trackerLine;
    std::array<double,16> currentPose;
    int phase=0;
    int phase_max=0;
    //added variables end
bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  ROS_INFO("INIT 2");
//added code segment1 start
  ifstream tracker_file("/home/dhrikarl/catkin_ws/rawXYZ_curveDiff_extended.txt");
  
  if (!tracker_file.is_open()) {
      cerr << "Could not open the file" << endl;
      return EXIT_FAILURE;
  }
  int j=0;
  while (std::getline(tracker_file, trackerLine)) {
      trackerLines.push_back(trackerLine);
      vector<double> trackerVector;
      istringstream ss(trackerLine);
      string word;
      std::array <double,3> trackerArray;
      int i=0;
      while (ss>> word){
        trackerArray[i]=std::stod(word);
        i++;
      
      }
      if (j==0){
        orgTrackerPose[0]=trackerArray[0];
        orgTrackerPose[1]=trackerArray[1];
        orgTrackerPose[2]=trackerArray[2];  
      }
      trackerVectors.push_back(trackerArray);
      j++;
      phase_max++;
      //ROS_INFO("Tracker %lf %lf %lf",trackerArray[0],trackerArray[1], trackerArray[2]);
      
  }
  tracker_file.close();
  ROS_INFO("phase_max %i", phase_max);

//added code segment1 end
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  
// original codes
  //double radius = 0.3;
  //double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  //double delta_x = radius * std::sin(angle);
  //double delta_z = radius * (std::cos(angle) - 1);
// original codes
  std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d; //robot desired position
  std::array<double, 16> command_pose = cartesian_pose_handle_->getRobotState().O_T_EE_c; //robot command position
  ROS_INFO("desired position %lf %lf %lf", new_pose[12], new_pose[13], new_pose[14] );
  ROS_INFO("commad position %lf %lf %lf", command_pose[12], command_pose[13], command_pose[14] );

  int a=phase;
  int b=phase+1;  
  if (b>=phase_max){a=phase_max-1;b=phase_max-1;}

  std::array<double,3> &trajPose1=reinterpret_cast<std::array<double,3>&>(trackerVectors.at(a));
  //std::array<double,3> &trajPose2=reinterpret_cast<std::array<double,3>&>(trackerVectors.at(b));
  //double delta_x=trajPose2[0]-trajPose1[0];
  //double delta_y=trajPose2[1]-trajPose1[1];
  //double delta_z=trajPose2[2]-trajPose1[2];
  double delta_x=trajPose1[1];
  double delta_y=trajPose1[0];
  double delta_z=trajPose1[2];
  ROS_INFO("delta position %lf %lf %lf", delta_x, delta_y, delta_z );
  //double delta_x=trajPose1[0]-new_pose[12];
  //double delta_y=trajPose1[1]-new_pose[13];
  //double delta_z=trajPose1[2]-new_pose[14];

//original codes
  new_pose[12] += delta_x;
  new_pose[13] += delta_y;
  new_pose[14] += delta_z;
//original codes

  franka_example_controllers::phase += 1;
  ROS_INFO("phase %i", phase);
  ROS_INFO("position %lf %lf %lf", new_pose[12], new_pose[13], new_pose[14] );
  cartesian_pose_handle_->setCommand(new_pose);
  std::array<double, 7> qvel = cartesian_pose_handle_->getRobotState().dq_d;
  std::array<double, 7> qacc = cartesian_pose_handle_->getRobotState().ddq_d;
  //ROS_INFO("joint velocity %lf %lf %lf %lf %lf %lf %lf", qvel[0], qvel[1], qvel[2], qvel[3], qvel[4], qvel[5], qvel[6]);
  //ROS_INFO("joint accleration %lf %lf %lf %lf %lf %lf %lf", qacc[0], qacc[1], qacc[2], qacc[3], qacc[4], qacc[5], qacc[6]);
}

  
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
