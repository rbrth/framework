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


#include "MoveToHomeState.h"


void MoveToHomeState::LeftEndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
     // The robot keeps triggering this callback at 100hz.
     // If bool is false, no response is given.
     if(check_cartesian_endpoints)
     {
          // Output position in left_gripper tf frame, relative to /base frame of robot.
          //ROS_INFO("Robot left position    (x,y,z)    : %.2f %.2f %.2f"     , endpoint_msg->pose.position.x, endpoint_msg->pose.position.y, endpoint_msg->pose.position.z);
          //ROS_INFO("Robot left orientation (x,y,z,w)  : %.2f %.2f %.2f %.2f", endpoint_msg->pose.orientation.x, endpoint_msg->pose.orientation.y, endpoint_msg->pose.orientation.z, endpoint_msg->pose.orientation.w);
                  
          robot_left_endpoint_pose.position.x = endpoint_msg->pose.position.x;
          robot_left_endpoint_pose.position.y = endpoint_msg->pose.position.y;
          robot_left_endpoint_pose.position.z = endpoint_msg->pose.position.z;

          robot_left_endpoint_pose.orientation.x = endpoint_msg->pose.orientation.x;          
          robot_left_endpoint_pose.orientation.y = endpoint_msg->pose.orientation.y;  
          robot_left_endpoint_pose.orientation.z = endpoint_msg->pose.orientation.z;  
          robot_left_endpoint_pose.orientation.w = endpoint_msg->pose.orientation.w;  
     }
}




void MoveToHomeState::RightEndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
     // The robot keeps triggering this callback at 100hz.
     // If bool is false, no response is given.
     if(check_cartesian_endpoints)
     {
          // Output position in right_gripper tf frame, relative to /base frame of robot.
          //ROS_INFO("Robot left position    (x,y,z)    : %.2f %.2f %.2f"     , endpoint_msg->pose.position.x, endpoint_msg->pose.position.y, endpoint_msg->pose.position.z);
         
          robot_right_endpoint_pose.position.x = endpoint_msg->pose.position.x;
          robot_right_endpoint_pose.position.y = endpoint_msg->pose.position.y;
          robot_right_endpoint_pose.position.z = endpoint_msg->pose.position.z;

          robot_right_endpoint_pose.orientation.x = endpoint_msg->pose.orientation.x;          
          robot_right_endpoint_pose.orientation.y = endpoint_msg->pose.orientation.y;  
          robot_right_endpoint_pose.orientation.z = endpoint_msg->pose.orientation.z;  
          robot_right_endpoint_pose.orientation.w = endpoint_msg->pose.orientation.w; 
     }
}




bool MoveToHomeState::calculateJointPositions_Baxter(std::string side, geometry_msgs::PoseStamped pose, std::vector<sensor_msgs::JointState> &joint_states)
{

     if(side == "left")
     {
          inverse_kinematics_client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
     }
     else
     {
          if(side == "right")
          {
               inverse_kinematics_client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
          }
          else
          {
               ROS_WARN("While calling getJointPositions, wrong side (std::string) was passed. Either pass 'left' or 'right'. Case sensitive.");
               return false;
          }
     }

     // Set up service message that will be send and received.
     baxter_core_msgs::SolvePositionIK service_message; 

     // The target pose is send as message.
     service_message.request.pose_stamp.push_back(pose);

     // Call client with target pose to get joint positions for that pose.
     if (inverse_kinematics_client .call(service_message))
     {
         //ROS_INFO("Succesful called IKService to get Joint positions.");
         if(service_message.response.isValid[0])
         {
               // Fill global variable with response.
               joint_states = service_message.response.joints;
               
               // Print joints names and their next target position.  
               /*for(int i=0;i<7;i++) 
               {
                   ROS_INFO("Name     : %s", joint_states[0].name[i].c_str());
                   ROS_INFO("Position : %f", joint_states[0].position[i]);
               }*/
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
         ROS_WARN("Failed to call IK Service to get joint positions.");
         return false;
     }
     return false;
}




void MoveToHomeState::MoveToHome_Baxter()
{
          ROS_INFO("Moving Baxter to home position.");
          
          double distance_to_goal;
          bool first = true;
          
          // When distance to target is smaller than 2 mm, then target is 'reached'. 
          // Baxter cannot go more accurately to all coordinates.
          double allowed_difference = 0.002;
          while(distance_to_goal > allowed_difference || first)
          {
               first = false;
              
		     // Define goal pose of home state.
		     // TODO: read in from launch file.    
		     geometry_msgs::PoseStamped goal_pose;   
               goal_pose.header.stamp        = ros::Time::now();
               goal_pose.header.frame_id     = "base";
               goal_pose.pose.position.x     =   0.86;  
               goal_pose.pose.position.y     =  -0.52; 
               goal_pose.pose.position.z     =   0.30;  
               goal_pose.pose.orientation.x  =   0.33;  
               goal_pose.pose.orientation.y  =   0.66;  
               goal_pose.pose.orientation.z  =  -0.33; 
               goal_pose.pose.orientation.w  =   0.66;  
               
               // Calculate current distance to goal. 
               distance_to_goal = sqrt( pow ((robot_right_endpoint_pose.position.x - goal_pose.pose.position.x),2) + pow((robot_right_endpoint_pose.position.y - goal_pose.pose.position.y),2) + pow((robot_right_endpoint_pose.position.z - goal_pose.pose.position.z),2) );    
               //ROS_INFO("Distance to goal : %.4f", distance_to_goal );

               // Use Baxters IK service to calculate joint positions for this pose.
               std::vector<sensor_msgs::JointState> joint_states;
               bool found_joints_solution = calculateJointPositions_Baxter("right", goal_pose, joint_states);
          
               if(found_joints_solution)
               {
                    // Inintialize message.
                    baxter_core_msgs::JointCommand joint_command_message;
                    
                    // Set mode to either 4 (raw mode) or mode 1 (position mode, preferred).
                    joint_command_message.mode = 1;
                    
                    // Fill message.
                    for(int i=0;i<7;i++) 
                    {
                         joint_command_message.names.push_back(joint_states[0].name[i]);
                         joint_command_message.command.push_back(joint_states[0].position[i]);
                    }             
                    
                    // Send message.  
                    robot_right_joint_command_publisher.publish(joint_command_message);
               }
          }
          if(distance_to_goal > allowed_difference)
          {
               transition = "goal reached";
          }
          else
          {
               transition = "not reached";
          }
}




MoveToHomeState::MoveToHomeState()
{
     // Constantly update global variables with endpoints of the robot.
     check_cartesian_endpoints = true;
     
     // Initially the goal is not reached.
     transition = "not reached";
     
     // Get home position from launch file.
     node_handle.getParam("home_position", home_position);
     
     // Publishers to set joint positions of robot.
     robot_left_joint_command_publisher  = node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);               
     robot_right_joint_command_publisher = node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);               
        
     // Callbacks checks to obtain current cartesian endpoint of both arms.
     robot_left_endpoint_state_subscriber  = node_handle.subscribe("/robot/limb/left/endpoint_state" , 1000, &MoveToHomeState::LeftEndpointStateCallback_Baxter, this); 
     robot_right_endpoint_state_subscriber = node_handle.subscribe("/robot/limb/right/endpoint_state", 1000, &MoveToHomeState::RightEndpointStateCallback_Baxter, this); 
}




std::string MoveToHomeState::execute(std::map<std::string, boost::any> * data)
{
     MoveToHome_Baxter();     
     return transition;
}
