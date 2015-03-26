/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,Umeå University
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
#include "MoveToHomeState.h"


MoveToHomeState::MoveToHomeState() 
{     
     // Initially the goal is not reached.
     transition = "goal not reached";
     
     // Get home position from launch file.
     node_handle.getParam("home_position", home_position);
     
     first = true;
}

// TODO: select arm from launch file parameter.
std::string MoveToHomeState::execute(std::map<std::string, boost::any> * data)
{        
    // Create the action client for arm movement. True causes the client to spin its own thread.
    actionlib::SimpleActionClient<robot_control::MoveArmAction> action_client_right("robot_control_right_arm", true); 

    ROS_INFO("Waiting for action server for moving rigth Baxter arm to start.");
    action_client_right.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server for moving rigth Baxter arm is started.");
         
    ROS_INFO("Sending goal.");
    robot_control::MoveArmGoal goal_message;
    goal_message.goal_pose.header.stamp    = ros::Time::now();
    goal_message.goal_pose.header.frame_id = "base";
    goal_message.goal_pose.pose.position.x = home_position[0];
    goal_message.goal_pose.pose.position.y = home_position[1];
    goal_message.goal_pose.pose.position.z = home_position[2];
    goal_message.goal_pose.pose.orientation.x = home_position[3];
    goal_message.goal_pose.pose.orientation.y = home_position[4];
    goal_message.goal_pose.pose.orientation.z = home_position[5];
    goal_message.goal_pose.pose.orientation.w = home_position[6];

    action_client_right.sendGoal(goal_message);
          
    actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::ACTIVE;
    while(state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE)
    {
        state = action_client_right.getState();
        //ROS_INFO("Action state: %s",state.toString().c_str());
    }

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (action_client_right.getResult()->goal_reached)
        {   
            ROS_INFO("Arm reached home position.");
            transition = "goal reached";
        }
        else
        {
            ROS_WARN("Could not move arm to home position.");
            transition = "goal not reached";
        }
    }
    else
    {
        if(state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("Could not move arm to home position. Action preempted.");
            transition = "goal not reached";
        }
        else
        {
            ROS_INFO("Home position not reached. Unhandled action state: %s",state.toString().c_str());
            transition = "error";
        }
    }

    return transition;
}
