/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2021, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <qb_device_control/qb_device_control.h>
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <qb_device_utils/ros_sigint_handler.h>
#include <iostream>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <math.h> 
#include <time.h>
#include <chrono>
#include <algorithm>
#include <unistd.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;
using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::vector;



int main(int argc, char** argv) {
//   ros::init(argc, argv, "qb_device_control", ros::init_options::NoSigintHandler);
  ros::init(argc,argv,"qb_device_control");
  ros::NodeHandle node_handle;
  ros::Rate rate(300); 
//   ros_sigint_handler::overrideHandlers();
  {
    ifstream hand_file("/home/dhrikarl/catkin_ws/Robothand.txt"); 
    
    if (!hand_file.is_open()) {
        cerr << "Could not open the file" << endl;
        return EXIT_FAILURE;
    }
    vector<string> handLines;
    vector<float*> handVectors;
    vector<int> handTime;
    string handLine;
    const char* comp = ":";
    const char* minus = "-";
    while (std::getline(hand_file, handLine)) {
        handLines.push_back(handLine);
        istringstream iss(handLine);
        vector<float> handVector;
        for (string s; iss >> s;)
            if (isdigit(s[0]) == 0) {
                if (s.find(comp) != std::string::npos) {
                    s.erase(s.begin());
                }
                if (isalpha(s[0]) == 0) {
                    if (s.find(minus)!= std::string::npos) {
                        handVector.push_back(float(0));
                    }
                    else {
                        handTime.push_back(std::stoi(s));
                    }
                }
                else {}
            }
            else {
                handVector.push_back(std::stof(s));
            }
        const int m = handVector.size();
        float* handArray = new float[m];
        copy(handVector.begin(), handVector.end(), handArray);
        handVectors.push_back(handArray);
    }
    hand_file.close();



    

    qb_device_control::qbDeviceControl qb_device_control;
    ros::Publisher trajectoryFollower = node_handle.advertise<trajectory_msgs::JointTrajectory>("/qbhand1/control/qbhand1_synergy_trajectory_controller/command",1000);
    
    trajectory_msgs::JointTrajectory joint_state;

    std::vector<trajectory_msgs::JointTrajectoryPoint> points_n(3);
    points_n[0].positions.resize(1);
    points_n[0].velocities.resize(1);
    points_n[0].accelerations.resize(1);
    points_n[0].effort.resize(1);
    points_n[0].positions[0] = 1; 
    points_n[0].velocities[0]=0; 
    points_n[0].accelerations[0]=0;
    points_n[0].effort[0]=0; 
    points_n[0].time_from_start= {secs: 1, nsecs: 0};
    
    clock_t reference = clock();
    int index = 0;
    // while(!ros_sigint_handler::isShuttingDown()) {
    while(ros::ok()) {

      double secs =ros::Time::now().toSec();
      joint_state.header.stamp = ros::Time::now();
      joint_state.joint_names.resize(1);
      joint_state.points.resize(1);
      joint_state.joint_names[0] ="qbhand1_synergy_joint";
      
    //   trajectory_msgs::JointTrajectoryPoint points_n;
    //   points_n.positions.push_back(*handVectors.at(index)/19000);
    //   joint_state.points.push_back(points_n);
      joint_state.points[0] = points_n[0];
      ROS_INFO("Position : %f", *handVectors.at(index));
      joint_state.points[0].positions[0]=*handVectors.at(index)/19000;
      joint_state.points[0].velocities[0]=1;
      ros::Duration time_from_header=ros::Time::now()-joint_state.header.stamp;
      joint_state.points[0].time_from_start=time_from_header;
      
      ROS_INFO("stream success, ");
      index++;
      trajectoryFollower.publish(joint_state);

      ros::spinOnce();
      rate.sleep();
    }
    // all the destructors are called before 'ros::shutdown()'
    // DO NOT remove brackets!
  }

  ros::requestShutdown();
  return 0;
}
