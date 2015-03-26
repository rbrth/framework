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

#include "ros/ros.h"
#include <visual_servo_control/request_servo_velocity_vector.h>
#include <vector>

int main(int argc, char **argv)
{
     ros::init(argc, argv, "cycle_node");
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<visual_servo_control::request_servo_velocity_vector>("/visual_servo_control_node/request_servo_velocity_vector");
  
     bool first = true;
     ros::Rate rate(10.0);
     while (n.ok())
     {
          std::string request_message;
          if(first)
          {
               request_message = "initialize";
               visual_servo_control::request_servo_velocity_vector srv;  
               srv.request.request_message = request_message; 
               client.call(srv);
               first = false;
          }
          else
          {
               request_message = "cycle";
               visual_servo_control::request_servo_velocity_vector srv;  
               srv.request.request_message = request_message;  
                                   
               if (client.call(srv))
               {                
                    std::vector<double> velocity_vector(srv.response.servo_velocity_vector );
		          ROS_INFO("Velocity vector has been recieved: %f %f %f %f %f %f", velocity_vector[0] , velocity_vector[1] , velocity_vector[2] , velocity_vector[3] , velocity_vector[4] , velocity_vector[5] );
               }
               else
               {
                   ROS_INFO("Could not call service.");
               }
          }
          ros::spinOnce();
          rate.sleep();
     }     
     return 0;
}
