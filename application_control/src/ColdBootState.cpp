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

#include "ColdBootState.h"


void ColdBootState::robotStateCallback(const baxter_core_msgs::AssemblyState::ConstPtr& state_msg)
{
     // If robot is already enabled, then do nothing. 
     // The robot keeps triggering this callback.
     if(!robot_state_enabled)
     {
          // If the robot is enabled, set global variable to true.
          if(state_msg->enabled)
          {
               ROS_INFO("Robot enabled");
               robot_state_enabled = true;
          }
          else
          {
               ROS_INFO("Robot not yet enabled, trying to enable.");
               robot_activation = node_handle.advertise<std_msgs::Bool>("/robot/set_super_enable", 1000);
               std_msgs::Bool activation;
               activation.data = true;
               robot_activation.publish(activation);
          }
     }
}



ColdBootState::ColdBootState()
{
    // System is not ready by default.
    robot_state_enabled = false;
         
    // Callback check if Baxter robot is activated.
    robot_state_subscriber = node_handle.subscribe("/robot/state", 1000, &ColdBootState::robotStateCallback, this);
}



std::string ColdBootState::execute(std::map<std::string, boost::any> * data)
{
    // Check if all ready bools are true.
    // If so, then the Coldboot is done.
    if(robot_state_enabled)
    {
          transition_message = "ready";
    }
    else
    {
          transition_message = "not ready";
    }
    ros::Rate rate(1.0);
    rate.sleep();

    transition_message = "ready";
    return transition_message; 
}
