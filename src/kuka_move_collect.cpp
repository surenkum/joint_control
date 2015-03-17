#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>
#include <boost/foreach.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "brics_actuator/JointPositions.h"

#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>

class Robot_control{
  // Node associated with the code
  ros::NodeHandle node;

  public:
  // Create a publisher to send data to robot arm
  ros::Publisher robot_arm;
  // Create a subscriber to write data to states
  ros::Subscriber joint_get_states;
  std::ofstream data_file; // Object to write data to the file
  void initParams(); // To initialize the object
  void write_data(const sensor_msgs::JointState::ConstPtr &msg);
  // Matrix of all the poses
  std::vector< std::vector<float> > pose_sequence;
};

void Robot_control::initParams(){
  robot_arm = node.advertise<brics_actuator::JointPositions> ("/arm_1/arm_controller/position_command",10);
  joint_get_states = node.subscribe("/joint_states",1000,&Robot_control::write_data,this);

  std::string file_path; 
  node.getParam("/joint_control_kuka/file_path",file_path); // Getting file leading path from the config file
  // Appending current time stamp to the file write path
  boost::posix_time::ptime thistime = boost::posix_time::second_clock::local_time();
  std::string ft_stamp = boost::posix_time::to_simple_string(thistime).c_str();
  std::string filename = file_path+ft_stamp+"joint_data.csv";
  // Open the file for writing data
  data_file.open(filename.c_str(),std::ios::trunc);

  // Getting the pose information from class file
  std::vector<float>pose_start; // Initial pose of the robot
  std::vector<float>pose_end; // Final pose of the robot

  // Read this data from .yaml file
  node.getParam("/joint_control_kuka/robot_pose_start",pose_start);
  node.getParam("/joint_control_kuka/robot_pose_end",pose_end);
  pose_sequence.push_back(pose_start);
  pose_sequence.push_back(pose_end);
  pose_sequence.push_back(pose_start); // To make the robot come back to the start position
}

void Robot_control::write_data(const sensor_msgs::JointState::ConstPtr &msg){
  // Write the data to the csv file
  if (data_file.is_open()){
    data_file<<msg->header.stamp.sec<<","<<msg->header.stamp.nsec<<","<<msg->position[0]<<","<<msg->position[1]<<","
      <<msg->position[2]<<","<<msg->position[3]<<","<<msg->position[4]<<","<<msg->velocity[0]<<","<<msg->velocity[1]<<","
      <<msg->velocity[2]<<","<<msg->velocity[3]<<","<<msg->velocity[4]<<std::endl;
  }
}

int main(int argc,char **argv){

  ros::init(argc,argv,"robot_control");
  Robot_control pose_drive_collect;
  // Initialize the robot control object
  pose_drive_collect.initParams();

  // Frequency at which these commands will be passed
  ros::Rate loop_rate(0.7); // 0.1 Hz rate

  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  std::stringstream jointName;

  while(ros::ok()){
    // Pass on all the pose commands to the robot manipulator
    for (unsigned int j = 0; j < pose_drive_collect.pose_sequence.size(); j++) {

      // ::io::base_unit_info <boost::units::si::angular_velocity>).name();
      armJointPositions.resize(pose_drive_collect.pose_sequence[j].size()); 
      for (unsigned int i = 0; i < pose_drive_collect.pose_sequence[j].size(); i++) {
        jointName.str("");
        jointName << "arm_joint_" << (i + 1);
        armJointPositions[i].joint_uri = jointName.str();
        armJointPositions[i].value = pose_drive_collect.pose_sequence[j][i];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
        std::cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << std::endl;

      }
      command.positions = armJointPositions;
      pose_drive_collect.robot_arm.publish(command);
      std::cout<<"Sent position command to the arm"<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();

    }
    ros::shutdown(); // Shut the node down
    pose_drive_collect.data_file.close(); // Close the file
  }
  return 0;
}
