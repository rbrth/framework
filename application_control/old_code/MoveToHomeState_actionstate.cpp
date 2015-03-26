/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,Umeå University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Open Source Robotics Foundation nor
* the names of its contributors may be used to endorse or promote
* products derived from this software without specific prior
* written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Ready state. Should wait in this state until the user starts the system
 * (i.e. do not begin to move the arm before the user says OK)
 */

#include "MoveToHomeState.h"


void MoveToHomeState::robotLeftEndpointStateCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
     // The robot keeps triggering this callback at 100hz.
     // If bool is false, no response is given.
     if(check_cartesian_endpoints)
     {
          // Output position in left_gripper tf frame, relative to /base frame of robot.
          ROS_INFO("Robot left position    (x,y,z)    : %.2f %.2f %.2f"     , endpoint_msg->pose.position.x, endpoint_msg->pose.position.y, endpoint_msg->pose.position.z);
          ROS_INFO("Robot left orientation (x,y,z,w)  : %.2f %.2f %.2f %.2f", endpoint_msg->pose.orientation.x, endpoint_msg->pose.orientation.y, endpoint_msg->pose.orientation.z, endpoint_msg->pose.orientation.w);
                  
          /*try
          {
               geometry_msgs::PoseStamped original_pose;
               geometry_msgs::PoseStamped transformed_pose;
               
               original_pose.pose = endpoint_msg->pose;
               original_pose.header.frame_id = "left_gripper";
               original_pose.header.stamp = ros::Time::now();
              
               transform_listener.transformPose("left_gripper", ros::Time::now(), original_pose, "base", transformed_pose);

               ROS_INFO("T Robot left postion (x,y,z)       : %.2f %.2f %.2f"     , transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z);
               ROS_INFO("T Robot left orientation (x,y,z,w) : %.2f %.2f %.2f %.2f", transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);
          
          }
          catch(tf::TransformException& transformException)
          {
               ROS_ERROR("Received an exception trying to transform a point.");
          }*/
          
          ros::Rate rate(1.0);
          rate.sleep();
     }
}




void MoveToHomeState::robotRightEndpointStateCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
     // The robot keeps triggering this callback at 100hz.
     // If bool is false, no response is given.
     if(check_cartesian_endpoints)
     {
          // Output position in right_gripper tf frame, relative to /base frame of robot.
          ROS_INFO("Robot right position    (x,y,z)    : %.2f %.2f %.2f"     , endpoint_msg->pose.position.x, endpoint_msg->pose.position.y, endpoint_msg->pose.position.z);
          ROS_INFO("Robot right orientation (x,y,z,w)  : %.2f %.2f %.2f %.2f", endpoint_msg->pose.orientation.x, endpoint_msg->pose.orientation.y, endpoint_msg->pose.orientation.z, endpoint_msg->pose.orientation.w);
          
          ros::Rate rate(1.0);
          rate.sleep();
     }
}




bool MoveToHomeState::getJointPositions(std::string side, geometry_msgs::PoseStamped pose)
{
     ros::ServiceClient client;
     if(side == "left")
     {
          client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
     }
     else
     {
          if(side == "right")
          {
               client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
          }
          else
          {
               ROS_WARN("While calling getJointPositions, wrong side (std::string) was passed. Either pass 'left' or 'right'. Case sensitive.");
               return false;
          }
     }

     baxter_core_msgs::SolvePositionIK service_message; 

     service_message.request.pose_stamp.push_back(pose);

     if (client.call(service_message))
     {
         ROS_INFO("Succesful called IKService to get Joint positions.");
         
         if(service_message.response.isValid[0])
         {
               joint_states = service_message.response.joints;
                    
               for(int i=0;i<7;i++) 
               {
                   ROS_INFO("Name     : %s", joint_states[0].name[i].c_str());
                   ROS_INFO("Position : %f", joint_states[0].position[i]);
               }
               return true;     
         }
         else
         {
               ROS_WARN("No valid joint solutions found.");
               return false;  
         }  
     }
     else
     {
         ROS_WARN("Failed to call IKService to get joint positions.");
         return false;
     }
     return false;
}



MoveToHomeState::MoveToHomeState() : SimpleActionState("/FollowJointTrajectoryAction")
{
     first_state_visit = true;

     // When this state is created, do not respond to robot output yet.
     check_cartesian_endpoints = false;

     // Callback checks to obtain current cartesian endpoint of both arms.
     robot_left_endpoint_state_subscriber  = node_handle.subscribe("/robot/limb/left/endpoint_state" , 1000, &MoveToHomeState::robotLeftEndpointStateCallback, this); 
     robot_right_endpoint_state_subscriber = node_handle.subscribe("/robot/limb/right/endpoint_state", 1000, &MoveToHomeState::robotRightEndpointStateCallback, this); 
}



std::string MoveToHomeState::execute(std::map<std::string, boost::any> * data)
{

     // When it is the first time this state is visited from a different state, the bool is true and
     // the following code is executed. If this state is recursively executing, the bool is false.
	if (first_state_visit)
	{
		first_state_visit = false;
		
		// Define goal pose of home state
		// TODO: read in from launch file.
          
          goal_pose.header.stamp        = ros::Time::now();
          goal_pose.header.frame_id     = "base";
          goal_pose.pose.position.x     =  0.67;
          goal_pose.pose.position.y     =  0.00;
          goal_pose.pose.position.z     =  0.17;
          goal_pose.pose.orientation.x  =  0.36;
          goal_pose.pose.orientation.y  =  0.93;
          goal_pose.pose.orientation.z  = -0.00;
          goal_pose.pose.orientation.w  =  0.06;

          // Use Baxters IK service to calculate joint positions for this pose.
          bool found_solution = getJointPositions("right", goal_pose);
          
          if(found_solution)
          {           
               trajectory_msgs::JointTrajectory joint_trajectory;

               joint_trajectory.header.stamp      = ros::Time::now();
               joint_trajectory.header.frame_id   = "base";

               for(int i=0;i<7;i++) 
               {
                   joint_trajectory.joint_names.push_back(joint_states[0].name[i);
                   joint_trajectory.points.push_back(joint_states[0].name[i);
                   
                   ROS_INFO("Name     : %s", joint_states[0].name[i].c_str());
                   ROS_INFO("Position : %f", joint_states[0].position[i]);
               }

               joint_trajectory.joint_names
               
               std::vector<sensor_msgs::JointState>    joint_states;
               
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
               
                Header header
                string[] joint_names
                   JointTrajectoryPoint[] points
               
               
               trajectory_msgs::JointTrajectory joint_trajectory;
          
               goal_.trajectory = 
          }
          
          joint_states
          
          
     }
     
     
     // Movement has started so we request the current state of the movement.
	// Depending on what is returned, action is undertaken.
	actionlib::SimpleClientGoalState current_state_ = ac_.getState();
	switch (current_state_.state_)
	{

	case actionlib::SimpleClientGoalState::SUCCEEDED:

		// Movement has finished.
		// Reset control parameters to initial value.
		first_ 			= true;
		collisionChecked 	= false;

		// Double check is goal is actually reached
		if (ac_.getResult()->reached_goal)
		{
			ROS_INFO("#GUI Arm moved into the workspace");
			state_transition = "PepperArmIntoWorkspace";
		}
		else
		{
			ROS_WARN("#GUI Arm could NOT move into workspace");
			state_transition = "Error";
		}
		break;

	case actionlib::SimpleClientGoalState::PREEMPTED:

		ROS_WARN("#GUI Arm could NOT move into workspace");
		state_transition = "Error";
		break;

	default:
		state_transition = "Moving";
		break;
	}
	return state_transition;
     
     
     
     transition = "goal reached";
     return transition;
}
