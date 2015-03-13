/*********************************************************************************************//**
* @file marker_control.cpp
*
* Simple node for testing marker control on sda10f
* 
* Copyright (c)
* Frantisek Durovsky
* Department of Robotics
* Technical University Kosice
* March 2015
*   
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/
#include <ros/ros.h>
#include <marker_control_lib.h>

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_control");
  ros::NodeHandle nh;

  Robot sda10f(&nh);

  ros::Subscriber marker_sub = nh.subscribe("marker3D_pose",1, &Robot::markerCallback, &sda10f);

  //Joint state subscribers
  ros::Subscriber r1_sub = nh.subscribe("/sda10f/sda10f_r1_controller/joint_states",10, &Robot::r1_Callback, &sda10f);
  ros::Subscriber r2_sub = nh.subscribe("/sda10f/sda10f_r2_controller/joint_states",10, &Robot::r2_Callback, &sda10f);
  ros::Subscriber b1_sub = nh.subscribe("/sda10f/sda10f_b1_controller/joint_states",10, &Robot::b1_Callback, &sda10f);
  ros::Subscriber b2_sub = nh.subscribe("/sda10f/sda10f_b2_controller/joint_states",10, &Robot::b2_Callback, &sda10f);

  ros::spin();

  /*  //Wait until everything starts up
  ros::Duration(10).sleep();
  


  
  std::cout << "Press Enter to start robot movemement" << std::endl;
  std::cin.get();
  
  //Small initial torso movement to power up servo controller
  std::vector<double> initial_torso_joint_values;
  torso.getCurrentState()->copyJointGroupPositions(torso.getCurrentState()->getRobotModel()->getJointModelGroup(torso.getName()), initial_torso_joint_values);
  for(size_t i = 0; i < initial_torso_joint_values.size(); i++)
    initial_torso_joint_values[i] -= 0.1;
  torso.setJointValueTarget(initial_torso_joint_values);
  torso.plan(torso_plan);  
  torso.execute(torso_plan);
  ros::Duration(3).sleep();
  
  //==========================================================
  //Move to zero position
  //==========================================================
  
  //Planning to a joint-space goal - zero torso position
  std::vector<double> torso_joint_values;
  torso.getCurrentState()->copyJointGroupPositions(torso.getCurrentState()->getRobotModel()->getJointModelGroup(torso.getName()), torso_joint_values);
  for(size_t i = 0; i < torso_joint_values.size(); i++)
    torso_joint_values[i] = 0;
  torso.setJointValueTarget(torso_joint_values);
  torso.plan(torso_plan);
  torso.execute(torso_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - zero arm_left positions
  std::vector<double> arm_left_joint_values;
  group_arm_left.getCurrentState()->copyJointGroupPositions(group_arm_left.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_left.getName()), arm_left_joint_values);
  for(size_t i = 0; i < arm_left_joint_values.size(); i++)
    arm_left_joint_values[i] = 0;
  group_arm_left.setJointValueTarget(arm_left_joint_values);
  group_arm_left.plan(arm_left_plan);
  group_arm_left.execute(arm_left_plan);
  ros::Duration(3).sleep();
  
  //Planning to a joint-space goal - zero arm_right positions
  std::vector<double> arm_right_joint_values;
  group_arm_right.getCurrentState()->copyJointGroupPositions(group_arm_right.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_right.getName()), arm_right_joint_values);
  for(size_t i = 0; i < arm_right_joint_values.size(); i++)
    arm_right_joint_values[i] = 0;
  group_arm_right.setJointValueTarget(arm_right_joint_values);
  group_arm_right.plan(arm_right_plan);
  group_arm_right.execute(arm_right_plan);
  ros::Duration(3).sleep();

  //============================================================  
  //Move left arm according to marker pose
  //============================================================
  std::vector<geometry_msgs::Pose> arm_left_cartesian_waypoints;
  moveit_msgs::RobotTrajectory arm_left_trajectory;


    //ROS_INFO_STREAM("Goal pose: " << goal_pose);
    
    //group_arm_left.setPoseTarget(goal_pose, group_arm_left.getEndEffectorLink());
    //group_arm_left.plan(arm_left_plan);
    //group_arm_left.execute(arm_left_plan);
    
    /*arm_left_cartesian_waypoints.clear();
    arm_left_cartesian_waypoints.push_back(goal_pose);
    
    double arm_left_fraction = group_arm_left.computeCartesianPath(arm_left_cartesian_waypoints, 0.02, 0, arm_left_trajectory, false);
    ROS_INFO("Cartesian path) (%.2f%% acheived)", arm_left_fraction * 100.0);
    
    //If computation was successfull move the arm
    if(arm_left_fraction == 1)
    {
      //Adding velocities to cartesian trajectory
      robot_trajectory::RobotTrajectory arm_left_rt_01(group_arm_left.getCurrentState()->getRobotModel(),"arm_left");
      arm_left_rt_01.setRobotTrajectoryMsg(*group_arm_left.getCurrentState(), arm_left_trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization arm_left_iptp;
      
      //Compute Time Stamps
      const double max_velocity_scaling_factor = 0.05;
      bool arm_left_success = arm_left_iptp.computeTimeStamps(arm_left_rt_01,max_velocity_scaling_factor);
      ROS_INFO("Computed time stamps: %s", arm_left_success ? "SUCCEDED" : "FAILED");

      //Get robot trajectory from RobotTrajectory
      arm_left_rt_01.getRobotTrajectoryMsg(arm_left_trajectory);
      arm_left_plan.trajectory_ = arm_left_trajectory;
      //ROS_INFO_STREAM("Trajectory" << arm_left_trajectory);
      group_arm_left.execute(arm_left_plan);    
      ros::Duration(3).sleep();
    }
    */

  return(EXIT_SUCCESS);
}




























