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

class Robot
{
public:
    Robot(ros::NodeHandle *nh);
    ~Robot();
    void markerCallback(const geometry_msgs::Pose::ConstPtr &msg);

private:
    ros::Publisher marker_pub;
    ros::Publisher robot_state_pub;

    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::JointModelGroup* joint_model_group_arm_left;
    moveit::core::RobotStatePtr kinematic_state;
    Eigen::Affine3d end_effector_state;

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Pose marker_pose;
    geometry_msgs::Pose goal_pose;

    std::string arm_left_end_effector,
                arm_right_end_effector;

    visualization_msgs::Marker marker;

    tf::TransformListener *listener;
    tf::StampedTransform marker_transform;

};



#endif //MARKER_CONTROL_LIB_H
