/*********************************************************************************************//**
* @file marker_control_lib.h
*
* Marker control library header
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
#ifndef MARKER_CONTROL_LIB_H
#define MARKER_CONTROL_LIB_H

/**********************************************************************************************/
//Standard ROS headers
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//Headers for direct Robot driver access
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//MoveIT Headers
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class Robot
{
public:
    Robot(ros::NodeHandle *nh);
    ~Robot();
    void markerCallback(const geometry_msgs::Pose::ConstPtr &msg);
    void move_groups_init(ros::NodeHandle nh);

    //Synchronized movement - direct driver access
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

    //JointState Callback - required for
    void r1_Callback(const sensor_msgs::JointState::ConstPtr &msg);
    void r2_Callback(const sensor_msgs::JointState::ConstPtr &msg);
    void b1_Callback(const sensor_msgs::JointState::ConstPtr &msg);
    void b2_Callback(const sensor_msgs::JointState::ConstPtr &msg);

    control_msgs::FollowJointTrajectoryGoal Trajectory_power_up();
    control_msgs::FollowJointTrajectoryGoal Trajectory_start();

private:
    //ROS messaging
    ros::Publisher marker_pub;
    ros::Publisher marker_trajectory_pub;
    ros::Publisher robot_state_pub;

    //TrajClient for direct motoman driver access
    TrajClient* traj_client_;

    //"Wait for message" flags for joint_states Callbacks
    bool r1_recieved, r2_recieved, b1_recieved, b2_recieved;

    //Joint State variables - holding current robot state
    sensor_msgs::JointState r1, r2, b1, b2;

    //Moveit declarations
    moveit::planning_interface::MoveGroup *group_arm_left;
    moveit::planning_interface::MoveGroup *group_arm_right;
    moveit::planning_interface::MoveGroup *group_torso;

    moveit::planning_interface::MoveGroup::Plan arm_left_plan;
    moveit::planning_interface::MoveGroup::Plan arm_right_plan;
    moveit::planning_interface::MoveGroup::Plan torso_plan;

    //ROS C++ API for FK and IK computations on current model
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::JointModelGroup* joint_model_group_arm_left;
    moveit::core::RobotStatePtr kinematic_state;
    Eigen::Affine3d end_effector_state;

    //Marker control realted poses
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Pose marker_pose;
    geometry_msgs::Pose goal_pose;

    //Visualization
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_trajectory_point;
    boost::circular_buffer<geometry_msgs::Point> *trajectory_circular_buffer;

    //TF variables
    tf::TransformListener *listener;
    tf::StampedTransform marker_transform;

};



#endif //MARKER_CONTROL_LIB_H
