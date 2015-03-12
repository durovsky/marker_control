/*********************************************************************************************//**
* @file marker_control_lib.cpp
*
* Marker control library
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
#ifndef MARKER_CONTROL_LIB_CPP
#define MARKER_CONTROL_LIB_CPP

#include <marker_control_lib.h>

Robot::Robot(ros::NodeHandle *nh)
{

    marker_pub = nh->advertise<visualization_msgs::Marker>("Aruco_marker_with_IK",1);
    robot_state_pub = nh->advertise<moveit_msgs::DisplayRobotState>("marker_goal_state",1);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group_arm_left = kinematic_model->getJointModelGroup("arm_left");

    listener = new tf::TransformListener();
}

Robot::~Robot()
{
    //delete ;
}

void
Robot::markerCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    marker_pose = *msg;

   //Get current marker tf
   listener->waitForTransform("base_link", "marker", ros::Time(0), ros::Duration(0.5));
   try
   {
        listener->lookupTransform("base_link", "marker", ros::Time(0), marker_transform);
   }
   catch (tf::TransformException &ex)
   {
       ROS_ERROR("%s", ex.what());
       ros::Duration(1.0).sleep();
   }

   const tf::Vector3 marker_origin = marker_transform.getOrigin();
   const tf::Quaternion marker_orientation = marker_transform.getRotation();

   goal_pose.position.x = marker_origin.getX();
   goal_pose.position.y = marker_origin.getY();
   goal_pose.position.z = marker_origin.getZ();

   goal_pose.orientation.x = marker_orientation.getX();
   goal_pose.orientation.y = marker_orientation.getY();
   goal_pose.orientation.z = marker_orientation.getZ();
   goal_pose.orientation.w = marker_orientation.getW();

   //Look for IK solution for current marker pose
   bool ik_found = kinematic_state->setFromIK(joint_model_group_arm_left, goal_pose, 3, 0.005);

   //---------------------------------------------------
   //Visualize marker in RViz
   marker.header.frame_id = "camera_optical_frame";
   marker.header.stamp = ros::Time::now();
   marker.ns = "basic_shapes";
   marker.id = 1;
   marker.type = visualization_msgs::Marker::CUBE;
   marker.action = visualization_msgs::Marker::ADD;

   marker.pose = marker_pose;
   marker.scale.x = 0.15;  marker.scale.y = 0.15;  marker.scale.z = 0.01;
   marker.lifetime = ros::Duration(0.2);

   //Marker color - Green for if IK exists, Red if no IK solution
   if (ik_found == true)
   {
     marker.color.r = 0.0f;  marker.color.g = 1.0f;  marker.color.b = 0.0f;  marker.color.a = 1.0;
   }
   else
   {
     marker.color.r = 1.0f;  marker.color.g = 0.0f;  marker.color.b = 0.0f;  marker.color.a = 1.0;
   }
   //Publish marker
   marker_pub.publish(marker);

   //---------------------------------------------------
   //Compute and publish Robot Goal State
   if(ik_found)
   {
      moveit_msgs::DisplayRobotState robot_state_msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, robot_state_msg.state);
      robot_state_pub.publish(robot_state_msg);
   }

}


#endif //MARKER_CONTROL_LIB_CPP
