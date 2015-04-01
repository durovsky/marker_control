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

Robot::Robot(ros::NodeHandle *nh) :
    r1_recieved(false),
    r2_recieved(false),
    b1_recieved(false),
    b2_recieved(false),
    trajectory_visualization(false),
    voice_control_active(false),
    voice_fix_position(false),
    voice_fix_orientation(false),
    simulation(true)

{
    //MoveIt groups initialization
    group_arm_left  = new moveit::planning_interface::MoveGroup ("arm_left");
    group_arm_right = new moveit::planning_interface::MoveGroup ("arm_right");
    group_torso     = new moveit::planning_interface::MoveGroup ("torso");
    move_groups_init(*nh);  //Read tolerance parameters from launch file

   //Direct driver access for synchronized sda10f movements
    traj_client_ = new TrajClient("/joint_trajectory_action", true);

    //if real robot connected wait for action server to come up
    if(simulation == false)
    {
        while(!traj_client_->waitForServer(ros::Duration(5.0)))
           ROS_INFO("Waiting for the trajectory_action server");

        ROS_INFO_STREAM("Joint trajectory action client initialized");
    }

    //Low level C++ approach for fast FK/IK computations
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group_arm_left  = kinematic_model->getJointModelGroup("arm_left");
    joint_model_group_arm_right = kinematic_model->getJointModelGroup("arm_right");

    //ROS messaging
    marker_pub             = nh->advertise<visualization_msgs::Marker>("Aruco_marker_with_IK",1);
    marker_trajectory_pub  = nh->advertise<visualization_msgs::Marker>("Aruco_marker_trajectory", 10);
    robot_state_pub        = nh->advertise<moveit_msgs::DisplayRobotState>("marker_goal_state",1);

    //Start TF listener
    listener = new tf::TransformListener();

    //Circular buffer for trajectory visualization
    trajectory_circular_buffer = new boost::circular_buffer<geometry_msgs::Point> (100);


}

Robot::~Robot()
{
    delete traj_client_;
    delete listener;
    delete trajectory_circular_buffer;
}

void
Robot::voiceCallback(const std_msgs::String::ConstPtr &msg)
{
    voice_command = msg->data;

    if(voice_command == "continue speech")      voice_control_active = true;
    if(voice_command == "pause speech")         voice_control_active = false;

    if(voice_command == "fix position")         voice_fix_position = true;
    if(voice_command == "release position")     voice_fix_position = false;
    if(voice_command == "fix orientation")      voice_fix_orientation = true;
    if(voice_command == "release orientation")  voice_fix_orientation = false;

    if(voice_command == "fix all")             {voice_fix_position = true;  voice_fix_orientation == true;  }
    if(voice_command == "release all")         {voice_fix_position = false; voice_fix_orientation == false; }

    if(voice_command == "basic_position")      startTrajectory(Trajectory_start());
    if(voice_command == "move all")            startTrajectory(Trajectory_current());
    if(voice_command == "move left arm")       move_left_arm();
    if(voice_command == "move right arm")      move_right_arm();

}

void
Robot::markerCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    current_marker = *msg;
    marker_pose = current_marker.pose;
    marker_id = current_marker.id;

    std::stringstream marker_id_string;
    marker_id_string << "marker_" << marker_id;

    //Get current marker tf
   listener->waitForTransform("base_link", marker_id_string.str(), ros::Time(0), ros::Duration(0.5));
   try
   {
        listener->lookupTransform("base_link", marker_id_string.str(), ros::Time(0), marker_transform);
   }
   catch (tf::TransformException &ex)
   {
       ROS_ERROR("%s", ex.what());
       ros::Duration(1.0).sleep();
   }

   const tf::Vector3 marker_origin = marker_transform.getOrigin();
   tf::Quaternion marker_orientation = marker_transform.getRotation();

   //Voice control of arm goal
   if(voice_fix_position == false)
   {
       goal_pose.position.x = marker_origin.getX();
       goal_pose.position.y = marker_origin.getY();
       goal_pose.position.z = marker_origin.getZ();
   }

   if(voice_fix_orientation == false)
   {
      goal_pose.orientation.x = marker_orientation.getX();
      goal_pose.orientation.y = marker_orientation.getY();
      goal_pose.orientation.z = marker_orientation.getZ();
      goal_pose.orientation.w = marker_orientation.getW();
   }

   bool ik_found;
   //Look for IK solution for current marker pose
   if(marker_id == arm_left_id)
     ik_found = kinematic_state->setFromIK(joint_model_group_arm_left, goal_pose, 3, 0.005);

   if (marker_id == arm_right_id)
     ik_found = kinematic_state->setFromIK(joint_model_group_arm_right, goal_pose, 3, 0.005);

   //---------------------------------------------------
   //Visualize marker pose in RViz
   marker.header.frame_id = "camera_optical_frame";
   marker.header.stamp = ros::Time::now();
   marker.ns = "basic_shapes";
   marker.id = 1;
   marker.type = visualization_msgs::Marker::CUBE;
   marker.action = visualization_msgs::Marker::ADD;

   marker.pose = marker_pose;
   marker.scale.x = 0.2;  marker.scale.y = 0.2;  marker.scale.z = 0.01;
   marker.lifetime = ros::Duration(0.2);

   //Marker color - Green for if IK exists, Red if no IK solution
   if (ik_found == true)
   {  marker.color.b = 1.0f;  marker.color.a = 1.0;  }
   else
   {  marker.color.r = 1.0f;  marker.color.a = 1.0;  }

   //Publish marker
   marker_pub.publish(marker);

   //Compute and publish Robot Goal State
   if(ik_found)
   {
      moveit_msgs::DisplayRobotState robot_state_msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, robot_state_msg.state);
      robot_state_pub.publish(robot_state_msg);
   }

   //--------------------------------------------------
   //Trajectory computation
   //--------------------------------------------------
   if(trajectory_visualization == true)
   {
       //Visualize marker trajectory in RViz
       marker_trajectory_point.header.frame_id = "camera_optical_frame";
       marker_trajectory_point.header.stamp = ros::Time::now();
       marker_trajectory_point.ns = "trajectory";
       marker_trajectory_point.id = 2;
       marker_trajectory_point.type = visualization_msgs::Marker::POINTS;
       marker_trajectory_point.action = visualization_msgs::Marker::ADD;

       marker_trajectory_point.scale.x = 0.005;  marker_trajectory_point.scale.y = 0.005;

       marker_trajectory_point.pose.orientation.w = 1;
       geometry_msgs::Point p;
       p.x = marker_pose.position.x;
       p.y = marker_pose.position.y;
       p.z = marker_pose.position.z;

       //Push current point to circular buffer
       trajectory_circular_buffer->push_back(p);
       marker_trajectory_point.points.clear();

       //Copy circular buffer to marker_trajectory_points
       for(size_t i = 0; i < trajectory_circular_buffer->size(); i++)
           marker_trajectory_point.points.push_back(trajectory_circular_buffer->at(i));

       marker_trajectory_point.lifetime = ros::Duration(5.0);

       //Marker color - Green for if IK exists, Red if no IK solution
       marker_trajectory_point.color.g = 1.0f; marker_trajectory_point.color.a = 1.0;

       //Publish marker trajectory
       marker_trajectory_pub.publish(marker_trajectory_point);
    }
}

void
Robot::move_groups_init(ros::NodeHandle nh)
{
 //==================================================
 //Read and set parameters from launch file
 //==================================================
 double planningTime;                    //default values
 double orientationTolerance;            //default values
 double positionTolerance;               //default values

 //Read parameters from launch file
 nh.getParam("simulation", simulation);
 nh.getParam("marker_control/planning_time", planningTime);
 nh.getParam("marker_control/orientation_tolerance", orientationTolerance);
 nh.getParam("marker_control/position_tolerance", positionTolerance);

 nh.getParam("marker_control/arm_right_id", arm_right_id);
 nh.getParam("marker_control/arm_left_id",  arm_left_id);

 ROS_INFO_STREAM("Simulation: " << simulation);
 ROS_INFO_STREAM("Arm left marker id: " << arm_left_id);
 ROS_INFO_STREAM("Arm right marker id: " << arm_right_id);
 ROS_INFO_STREAM("Planning time: " << planningTime);
 ROS_INFO_STREAM("Orientation tolerance: " << orientationTolerance);
 ROS_INFO_STREAM("Position tolerance: " << positionTolerance);

 //Set moveit_planning_interface parameters
 group_arm_left->setPlanningTime(planningTime);
 group_arm_left->setGoalOrientationTolerance(orientationTolerance);
 group_arm_left->setGoalPositionTolerance(positionTolerance);

 group_arm_right->setPlanningTime(planningTime);
 group_arm_right->setGoalOrientationTolerance(orientationTolerance);
 group_arm_right->setGoalPositionTolerance(positionTolerance);

 group_torso->setPlanningTime(planningTime);
 group_torso->setGoalOrientationTolerance(orientationTolerance);
 group_torso->setGoalPositionTolerance(positionTolerance);
}

////////////////////////////////////////////////////////////////////////////////////////
//Joint State Callbacks
//Saving current joint state values and setting up the initial flag
void
Robot::r1_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   r1.position = msg->position;
   if(r1_recieved == false) r1_recieved = true;
}

void
Robot::r2_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   r2.position = msg->position;
   if(r2_recieved == false) r2_recieved = true;
}

void
Robot::b1_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   b1.position = msg->position;
   if (b1_recieved == false) b1_recieved = true;
}

void
Robot::b2_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   b2.position = msg->position;
   if(b2_recieved == false) b2_recieved = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

//Trajecotry_power_up  - initial small robot movement to power up servo controller
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_power_up()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

    //Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(2);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0] + 0.05;
    goal.trajectory.points[ind].positions[1]  = r1.position[1] + 0.05;
    goal.trajectory.points[ind].positions[2]  = r1.position[2] + 0.05;
    goal.trajectory.points[ind].positions[3]  = r1.position[3] + 0.05;
    goal.trajectory.points[ind].positions[4]  = r1.position[4] + 0.05;
    goal.trajectory.points[ind].positions[5]  = r1.position[5] + 0.05;
    goal.trajectory.points[ind].positions[6]  = r1.position[6] + 0.05;
    goal.trajectory.points[ind].positions[7]  = r2.position[0] + 0.05;
    goal.trajectory.points[ind].positions[8]  = r2.position[1] + 0.05;
    goal.trajectory.points[ind].positions[9]  = r2.position[2] + 0.05;
    goal.trajectory.points[ind].positions[10] = r2.position[3] + 0.05;
    goal.trajectory.points[ind].positions[11] = r2.position[4] + 0.05;
    goal.trajectory.points[ind].positions[12] = r2.position[5] + 0.05;
    goal.trajectory.points[ind].positions[13] = r2.position[6] + 0.05;
    goal.trajectory.points[ind].positions[14] = b1.position[0] + 0.05;
    goal.trajectory.points[ind].positions[15] = b2.position[0] + 0.05;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    //return the goal
    return goal;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

//Trajecotry start
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_start()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

//Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(2);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0] = -1.2;
    goal.trajectory.points[ind].positions[1] = 0.82;
    goal.trajectory.points[ind].positions[2] = 1.16;
    goal.trajectory.points[ind].positions[3] = -1.55;
    goal.trajectory.points[ind].positions[4] = -0.85;
    goal.trajectory.points[ind].positions[5] = -1.23;
    goal.trajectory.points[ind].positions[6] = 1.8;
    goal.trajectory.points[ind].positions[7] = -1.2;
    goal.trajectory.points[ind].positions[8] = 0.82;
    goal.trajectory.points[ind].positions[9] = 1.16;
    goal.trajectory.points[ind].positions[10] = -1.55;
    goal.trajectory.points[ind].positions[11] = -0.85;
    goal.trajectory.points[ind].positions[12] = -1.23;
    goal.trajectory.points[ind].positions[13] = 1.8;
    goal.trajectory.points[ind].positions[14] = 1.57;
    goal.trajectory.points[ind].positions[15] = 1.57;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    //return the goal
    return goal;

}

//Trajecotry current - move to position defined by marker pose
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_current()
{
    ROS_INFO_STREAM("Moving arm callback");

    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(2);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = 0;
    goal.trajectory.points[ind].positions[1]  = 0;
    goal.trajectory.points[ind].positions[2]  = 0;
    goal.trajectory.points[ind].positions[3]  = 0;
    goal.trajectory.points[ind].positions[4]  = 0;
    goal.trajectory.points[ind].positions[5]  = 0;
    goal.trajectory.points[ind].positions[6]  = 0;
    goal.trajectory.points[ind].positions[7]  = 0;
    goal.trajectory.points[ind].positions[8]  = 0;
    goal.trajectory.points[ind].positions[9]  = 0;
    goal.trajectory.points[ind].positions[10] = 0;
    goal.trajectory.points[ind].positions[11] = 0;
    goal.trajectory.points[ind].positions[12] = 0;
    goal.trajectory.points[ind].positions[13] = 0;
    goal.trajectory.points[ind].positions[14] = 0;
    goal.trajectory.points[ind].positions[15] = 0;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);

    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;

    //Get joint values from current kinematic state
    std::vector<double> arm_left_joint_values;
    std::vector<double> arm_right_joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group_arm_left, arm_left_joint_values);
    kinematic_state->copyJointGroupPositions(joint_model_group_arm_right, arm_right_joint_values);

    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0] = arm_left_joint_values[0];
    goal.trajectory.points[ind].positions[1] = arm_left_joint_values[1];
    goal.trajectory.points[ind].positions[2] = arm_left_joint_values[2];
    goal.trajectory.points[ind].positions[3] = arm_left_joint_values[3];
    goal.trajectory.points[ind].positions[4] = arm_left_joint_values[4];
    goal.trajectory.points[ind].positions[5] = arm_left_joint_values[5];
    goal.trajectory.points[ind].positions[6] = arm_left_joint_values[6];
    goal.trajectory.points[ind].positions[7] = arm_right_joint_values[0];
    goal.trajectory.points[ind].positions[8] = arm_right_joint_values[1];
    goal.trajectory.points[ind].positions[9] = arm_right_joint_values[2];
    goal.trajectory.points[ind].positions[10] = arm_right_joint_values[3];
    goal.trajectory.points[ind].positions[11] = arm_right_joint_values[4];
    goal.trajectory.points[ind].positions[12] = arm_right_joint_values[5];
    goal.trajectory.points[ind].positions[13] = arm_right_joint_values[6];
    goal.trajectory.points[ind].positions[14] = 0;
    goal.trajectory.points[ind].positions[15] = 0;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    //return the goal
    return goal;
}

//Sends the command to start a given trajectory
void
Robot::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
   goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(2.0);
   traj_client_->sendGoal(goal);
}

void
Robot::move_left_arm(void)
{
    ROS_INFO_STREAM("Move arm callback");
    std::vector<double> arm_left_joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group_arm_left, arm_left_joint_values);

    group_arm_left->setJointValueTarget(arm_left_joint_values);
    group_arm_left->plan(arm_left_plan);
    //group_arm_left->execute(arm_left_plan);
    ros::Duration(3).sleep();
}

void
Robot::move_right_arm(void)
{
    ROS_INFO_STREAM("Move arm callback");
    std::vector<double> arm_right_joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group_arm_right, arm_right_joint_values);

    group_arm_right->setJointValueTarget(arm_right_joint_values);
    ROS_INFO_STREAM("Planning started");
    group_arm_right->plan(arm_right_plan);
    ROS_INFO_STREAM("Planning finished");
    //group_arm_right->execute(arm_right_plan);
    ros::Duration(3).sleep();
}


#endif //MARKER_CONTROL_LIB_CPP
