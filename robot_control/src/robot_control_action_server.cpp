/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,DLO, Wageningen University & Research Center.
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


#include <robot_control/MoveArmAction.h>
#include <actionlib/server/simple_action_server.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "baxter_core_msgs/EndpointState.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/JointCommand.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Quaternion.h" 
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <math.h> 
#include <XmlRpcValue.h>
#include "robot_control/request_endpointstate.h"



/*
*   Global variables.
*/
typedef actionlib::SimpleActionServer<robot_control::MoveArmAction> Server;
bool                                    running;
std::string                             left_or_right;
geometry_msgs::PoseStamped              robot_endpoint_pose;
geometry_msgs::PoseStamped              goal_pose;
ros::Subscriber                         robot_endpoint_state_subscriber;
ros::Publisher                          robot_joint_command_publisher;
ros::Publisher                          robot_joint_speed_publisher; 
ros::ServiceClient                      inverse_kinematics_client;
ros::ServiceServer                      endpoint_state_client;
ros::Subscriber                         servo_setgoal_subscriber;
ros::Subscriber                         servo_startstop_subscriber;
ros::Subscriber                         servo_execute_subscriber;



/*
*   Calculates next valid joint positions of the robot given a target pose using the Inverse Kinematics service and returns true.
*   If no valid joint positions were found, it returns with false.
*/
bool calculateJointPositions_Baxter(std::string side, geometry_msgs::PoseStamped pose, std::vector<sensor_msgs::JointState> &joint_states)
{
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
         ROS_WARN("Failed to call IKService to get joint positions.");
         ros::Duration(1.0).sleep();
         return false;
     }
     return false;
}





/*
*   Callback function for updating the current pose of the endpoint of the robot's arm.
*/
void EndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
          /*
          if(left_or_right == "left")
          {
              robot_endpoint_pose.header.frame_id = "left_hand" ;
          }
          else
          {
              if(left_or_right == "right")
              { 
                  robot_endpoint_pose.header.frame_id = "right_hand" ;
              }
              else
              {
                  robot_endpoint_pose.header.frame_id = "base";
                  ROS_WARN("In EndpointStateCallback_Baxter, left or right endpoint was not yet specified.");
              }
          }*/ 

          // The endpoint is defined as the pose of the gripper in the /base frame.
          robot_endpoint_pose.header.frame_id = "base";
          robot_endpoint_pose.header.stamp = ros::Time::now(); 

          robot_endpoint_pose.pose.position.x = endpoint_msg->pose.position.x;
          robot_endpoint_pose.pose.position.y = endpoint_msg->pose.position.y;
          robot_endpoint_pose.pose.position.z = endpoint_msg->pose.position.z;

          robot_endpoint_pose.pose.orientation.x = endpoint_msg->pose.orientation.x;          
          robot_endpoint_pose.pose.orientation.y = endpoint_msg->pose.orientation.y;  
          robot_endpoint_pose.pose.orientation.z = endpoint_msg->pose.orientation.z;  
          robot_endpoint_pose.pose.orientation.w = endpoint_msg->pose.orientation.w;
}





/*
*   Used for method #1 : continous movement.
*   
*   Callback to start or stop movement.
*/
void startStopCallback(const std_msgs::Bool::ConstPtr& running_msg)
{
    running = running_msg->data;

    if(running)
    {
      // Set robot speed [0-1].
      std_msgs::Float64 speed;
      speed.data = 0.10;    
      robot_joint_speed_publisher.publish(speed);
    }

}




/*
*   Used for method #1 : continous movement.
*   
*   Callback to update goal during movement.
*/
void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& servo_goal_msg)
{ 
    goal_pose.header.stamp        =  ros::Time::now();
    goal_pose.header.frame_id     =  servo_goal_msg->header.frame_id;
    goal_pose.pose.position.x     =  servo_goal_msg->pose.position.x;
    goal_pose.pose.position.y     =  servo_goal_msg->pose.position.y; 
    goal_pose.pose.position.z     =  servo_goal_msg->pose.position.z;  
    goal_pose.pose.orientation.x  =  servo_goal_msg->pose.orientation.x;  
    goal_pose.pose.orientation.y  =  servo_goal_msg->pose.orientation.y;    
    goal_pose.pose.orientation.z  =  servo_goal_msg->pose.orientation.z;   
    goal_pose.pose.orientation.w  =  servo_goal_msg->pose.orientation.w;
}




/*
*   Used for method #1 : continous movement.
*   
*   Callback to execute movement until start/stop callback above is called.
*/
void servoToGoalCallback(const std_msgs::Bool::ConstPtr& trigger_msg)
{
    // Will run indefinetely until callback flips the global running variable.
    while(running)
    {
       // Use Baxters IK service to calculate joint positions for globally saved pose.
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
          robot_joint_command_publisher.publish(joint_command_message);
       }

       // Check all callbacks, otherwise this node will be blocked from communication 
       // and cannot be turned off from outside this node.
       ros::spinOnce();
       ros::Duration(0.2).sleep();
    }
}






/*
*   Used for method #2 : single movement with action server.
*
*   Executes a goal pose that is received from the action server and gives
*   feedback on the current status.
*/
void execute(const robot_control::MoveArmGoalConstPtr& goal, Server* action_server)
{
          ROS_INFO("Moving Baxter to position.");

          // Set robot speed [0-1].
          std_msgs::Float64 speed;
          speed.data = 0.10;    
          robot_joint_speed_publisher.publish(speed);

          // When distance to target is smaller than 2 mm, then target is 'reached'. 
          // Baxter cannot go more accurately to all coordinates.
          double  distance_to_goal;
          double  allowed_difference  = 0.005;
          bool    success             = false;
          bool    preempted           = false;
          bool    first               = true;

          ros::Time start_time = ros::Time::now();

          while( (distance_to_goal > allowed_difference || first) && (!preempted) && (ros::Time::now() < start_time + ros::Duration(15)))
          {
               first = false;

               // Check if other nodes want to cancel this movement through the action server.
               if (action_server->isPreemptRequested() || !ros::ok())
               {
                    ROS_INFO("Action server of %s preempted", ros::this_node::getName().c_str());
                    action_server->setPreempted();
                    success   = false;
                    preempted = true;
               }
               else
               {
                    // Create goal message from recieved action message.
                    geometry_msgs::PoseStamped goal_pose;   
                    goal_pose.header.stamp        =  ros::Time::now();
                    goal_pose.header.frame_id     =  goal->goal_pose.header.frame_id;
                    goal_pose.pose.position.x     =  goal->goal_pose.pose.position.x;
                    goal_pose.pose.position.y     =  goal->goal_pose.pose.position.y; 
                    goal_pose.pose.position.z     =  goal->goal_pose.pose.position.z;  
                    goal_pose.pose.orientation.x  =  goal->goal_pose.pose.orientation.x;  
                    goal_pose.pose.orientation.y  =  goal->goal_pose.pose.orientation.y;    
                    goal_pose.pose.orientation.z  =  goal->goal_pose.pose.orientation.z;   
                    goal_pose.pose.orientation.w  =  goal->goal_pose.pose.orientation.w;    
                    
                    // Calculate current distance to goal and publish this as feedback.
                    distance_to_goal = sqrt( pow ((robot_endpoint_pose.pose.position.x - goal_pose.pose.position.x),2) + pow((robot_endpoint_pose.pose.position.y - goal_pose.pose.position.y),2) + pow((robot_endpoint_pose.pose.position.z - goal_pose.pose.position.z),2) );    
                    robot_control::MoveArmFeedback feedback_message;
                    feedback_message.distance_to_goal = distance_to_goal;
                    action_server->publishFeedback(feedback_message);
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
                         
                         // Send message to robot.  
                         robot_joint_command_publisher.publish(joint_command_message);
                    }
                    success = true;
                }
          }
          
          // If preemted or reached goal, then send the result.     
          robot_control::MoveArmResult result;
          if(distance_to_goal < allowed_difference && success)
          {
               result.goal_reached = true;
               action_server->setSucceeded(result);
               ROS_INFO("Baxter goal position reached.");
          }
          else
          {
              if(preempted)
              {
                  result.goal_reached = false;
                  action_server->setSucceeded(result);
                  ROS_INFO("Baxter goal position reached by preemption.");
              }
              else
              {
                  result.goal_reached = false;
                  action_server->setSucceeded(result);
                  ROS_INFO("Baxter goal position not reached.");
              }
          }
}





/* 
*   Service to return the most recent stamped pose of the robot's endpoint.
*/
bool getEndPointState(robot_control::request_endpointstate::Request &req, robot_control::request_endpointstate::Response &res)
{
    res.pose = robot_endpoint_pose;
    return true;
}





/*
*   Main initialization function of this node. 
*   Creates a left or right action server depending on launch file parameter.
*   Subscribes to a topic to let Baxter's arm move to a pose.
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle node_handle;
  
  // Get argument passed in launch file to determine if this
  // node controls the left or right arm of Baxter.
  left_or_right = argv[1];

  running = false;

  servo_setgoal_subscriber    = node_handle.subscribe("/servo_set_goal", 1000, &setGoalCallback);
  servo_startstop_subscriber  = node_handle.subscribe("/servo_startstop", 1000, &startStopCallback);
  servo_execute_subscriber    = node_handle.subscribe("/servo_execute", 1000, &servoToGoalCallback);
                
  // Callbacks checks to continously obtain current cartesian endpoint of both arms.
  if(left_or_right == "right")
  {
     robot_endpoint_state_subscriber = node_handle.subscribe("/robot/limb/right/endpoint_state", 1000, &EndpointStateCallback_Baxter);
     inverse_kinematics_client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");        
     robot_joint_command_publisher = node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
     robot_joint_speed_publisher = node_handle.advertise<std_msgs::Float64>("/robot/limb/right/set_speed_ratio", 1000);
     endpoint_state_client = node_handle.advertiseService("/request_right_endpoint_state", getEndPointState); 
  }
  else
  {
     if(left_or_right == "left")
     {
          robot_endpoint_state_subscriber  = node_handle.subscribe("/robot/limb/left/endpoint_state" , 1000, &EndpointStateCallback_Baxter); 
          inverse_kinematics_client = node_handle.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
          robot_joint_command_publisher  = node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);               
          robot_joint_speed_publisher = node_handle.advertise<std_msgs::Float64>("/robot/limb/left/set_speed_ratio", 1000);
          endpoint_state_client = node_handle.advertiseService("/request_left_endpoint_state", getEndPointState); 
     }
     else
     {
          ROS_ERROR("Could not initialize endpoint state subscriptions and set up inverser kinematics client. Wrong argument passed in launch file : %s . Should be 'left' or 'right'.", argv[0] );
     }
  }

  Server server(node_handle, ros::this_node::getName().c_str(), boost::bind(&execute, _1, &server), false);
  server.start();
  
  ros::spin();
  return 0;
}
