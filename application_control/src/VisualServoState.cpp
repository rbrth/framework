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
#include "VisualServoState.h"



/*
*   Corrects the robot arm once in the x and y direction in the end-effector frame.
*   Furthermore, the arm is moved forward.
*/
void VisualServoState::moveArm(std::vector<double> velocity_vector, float error_x, float error_y, std_msgs::Time image_time)
{
    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped transformed_pose;
 
    try
    {      
        // Create the source pose that is to be transformed.
        // This represents the offset pose in the gripper frame that
        // will be tranformed to the base frame. 
        // Thereby the new transformed pose will be shifted in regard to 
        // its original frame in base frame coordinates.
        source_pose.header.frame_id = "/right_gripper";
        source_pose.header.stamp    = image_time.data;         
        source_pose.pose.position.x = 0.0 - velocity_vector[0] / 2; //- error_y;
        source_pose.pose.position.y = 0.0 - velocity_vector[1] / 2; //- error_x;
        source_pose.pose.position.z = 0.0 + forward_distance_step ;
        source_pose.pose.orientation.x = 1.0;
        source_pose.pose.orientation.y = 0.0;
        source_pose.pose.orientation.z = 0.0;
        source_pose.pose.orientation.w = 0.0;
           
        // Keep track of traveled distance during servoing.
        traveled_servo_distance = traveled_servo_distance + forward_distance_step;

        // Wait (max 3 sec) until transform comes available in the system. There are always slight delays.
        //transform_listener.waitForTransform("/base", "/right_gripper", image_time.data, ros::Duration(3.0));

        transform_listener.waitForTransform("/base", "/right_gripper", ros::Time::now(), ros::Duration(3.0));

        // Transfrom original pose in /right gripper frame (at time defined in original pose) to /base frame (at current time).
        transform_listener.transformPose("/base", ros::Time::now(), source_pose, "/right_gripper", transformed_pose); 

        //ROS_INFO("Gripper postion (x,y,z)       : %.2f %.2f %.2f"     , robot_endpoint_pose.position.x, robot_endpoint_pose.position.y, robot_endpoint_pose.position.z);
        //ROS_INFO("Gripper orientation (x,y,z,w) : %.2f %.2f %.2f %.2f", robot_endpoint_pose.orientation.x, robot_endpoint_pose.orientation.y, robot_endpoint_pose.orientation.z, robot_endpoint_pose.orientation.w);    
        //ROS_INFO("Transformed Robot postion (x,y,z)       : %.2f %.2f %.2f"     , transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z);
        //ROS_INFO("Transformed Robot orientation (x,y,z,w) : %.2f %.2f %.2f %.2f", transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);        

        // Creating new goal message.
        geometry_msgs::PoseStamped goal_message;
        goal_message.header.frame_id = "base";
        goal_message.header.stamp = ros::Time::now();
        goal_message.pose.position.x = transformed_pose.pose.position.x;
        goal_message.pose.position.y = transformed_pose.pose.position.y;
        goal_message.pose.position.z = transformed_pose.pose.position.z;
        goal_message.pose.orientation.x = robot_endpoint_pose.pose.orientation.x;
        goal_message.pose.orientation.y = robot_endpoint_pose.pose.orientation.y;
        goal_message.pose.orientation.z = robot_endpoint_pose.pose.orientation.z;
        goal_message.pose.orientation.w = robot_endpoint_pose.pose.orientation.w;

        ROS_INFO("Updating visual servo goal.");
        servo_setgoal_publisher.publish(goal_message);

        // If the servo loop in the motion control node was not running yet, trigger it to do so.
        if(!external_servo_loop_running)
        {
            std_msgs::Bool trigger;
            trigger.data = true;
            servo_execute_publisher.publish(trigger);
            servo_startstop_publisher.publish(trigger);
            external_servo_loop_running = true;
        }        
    }
    catch(tf::TransformException& transformException)
    {
        ROS_ERROR("Received an exception trying to transform new pose for visual servo control. Retrying..");
        ROS_ERROR("Error details : %s ", transformException.what()); 
    }
}





/*
*   Service client function to request to endpoint state of the robot.
*   This information is needed to save the pose that corresponds to 
*   the view with the most recognized pepper.
*
*   TODO: This should actually be requested from the Application Control node 
*   and not be directly requested from the robot. 
*/
geometry_msgs::PoseStamped VisualServoState::requestEndPointState()
{
    std::string request_message = "trigger";
    robot_control::request_endpointstate service;  
    service.request.request_message = request_message; 
    if (robot_endpointstate_client.call(service))
    {
        return service.response.pose;
    }
    else
    {
        ROS_INFO("Could not request endpoint state of the robot.");
    }  
}





/*
*   Callback function for updating the current pose of the endpoint of the robot's arm.
*/
void VisualServoState::EndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
{
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
*   Constructor for this ROS node.
*/
VisualServoState::VisualServoState() 
{     
    // Initially the goal is not reached.
    transition = "goal not reached";

    // Client for requesting the endpoint state of the robot. Used to track which pose detects the most pepper.
    robot_endpointstate_client = node_handle.serviceClient<robot_control::request_endpointstate>("/request_right_endpoint_state");

    // Clients for requesting an image.
    image_client_rgb  = node_handle.serviceClient<image_acquisition::request_image>("/image_acquisition_node/request_image_rgb");
    image_client_gray = node_handle.serviceClient<image_acquisition::request_image>("/image_acquisition_node/request_image_gray");

    // Client for requesting detected fruit features from image.
    request_features_client = node_handle.serviceClient<fruit_detection::request_features>("/fruit_detection_node/request_features");

    // Client for requesting the visual servo velocity vector.
    servo_velocity_vector_client = node_handle.serviceClient<visual_servo_control::request_servo_velocity_vector>("/visual_servo_control_node/request_servo_velocity_vector");

    // Callback for continously updating the endpoint.
    robot_endpoint_state_subscriber = node_handle.subscribe("/robot/limb/right/endpoint_state", 1000, &VisualServoState::EndpointStateCallback_Baxter, this);

    // Publisher to cancel goal movement (preemption).
    cancel_robot_movement_publisher = node_handle.advertise<actionlib_msgs::GoalID>("/robot_control_right_arm/cancel", 1);

    // General publisher of current goal.
    servo_setgoal_publisher   = node_handle.advertise<geometry_msgs::PoseStamped>("/servo_set_goal", 1);
    servo_startstop_publisher = node_handle.advertise<std_msgs::Bool>("/servo_startstop", 1);
    servo_execute_publisher   = node_handle.advertise<std_msgs::Bool>("/servo_execute", 1);

    // Keep track of traveled distance during servoing.
    traveled_servo_distance = 0.00;

    // Amount in meters of movement towards the fruit per visual servo cycle.
    forward_distance_step = 0.025;

    // Whether servo loop is running on the action server (external) and on this node (internal).
    internal_servo_loop_running = false;
    external_servo_loop_running = false;
}





/*
*   Execution function of this ROS node. 
*   Moves arm to visual servo start pose after the servol control loop is activated.
*/
std::string VisualServoState::execute(std::map<std::string, boost::any> * data)
{        
    // Get starting point for visual servo control from previous state and go there.
    geometry_msgs::PoseStamped visual_servo_start_pose = boost::any_cast<geometry_msgs::PoseStamped>((*data)["visual_servo_start_pose"]);

    // Create the action client for arm movement. True causes the client to spin its own thread.
    actionlib::SimpleActionClient<robot_control::MoveArmAction> action_client_right("robot_control_right_arm", true); 

    ROS_INFO("Waiting for action server for moving right Baxter arm to start.");
    // Will wait for infinite time until action server is started.
    action_client_right.waitForServer(); 
    ROS_INFO("Action server for moving right Baxter arm is started.");
         
    ROS_INFO("Sending goal.");
    robot_control::MoveArmGoal goal_message;
    goal_message.goal_pose.header.stamp    = ros::Time::now();
    goal_message.goal_pose.header.frame_id = visual_servo_start_pose.header.frame_id;
    goal_message.goal_pose.pose.position.x = visual_servo_start_pose.pose.position.x;
    goal_message.goal_pose.pose.position.y = visual_servo_start_pose.pose.position.y;
    goal_message.goal_pose.pose.position.z = visual_servo_start_pose.pose.position.z;
    goal_message.goal_pose.pose.orientation.x = visual_servo_start_pose.pose.orientation.x;
    goal_message.goal_pose.pose.orientation.y = visual_servo_start_pose.pose.orientation.y;
    goal_message.goal_pose.pose.orientation.z = visual_servo_start_pose.pose.orientation.z;
    goal_message.goal_pose.pose.orientation.w = visual_servo_start_pose.pose.orientation.w;

    action_client_right.sendGoal(goal_message);
          
    actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::ACTIVE;
    while(state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      //ROS_INFO("Action state: %s",state.toString().c_str());
      state = action_client_right.getState();   
    }

    bool success = false;
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (action_client_right.getResult()->goal_reached)
        {   
            ROS_INFO("Arm reached visual servo start position.");
            success = true;
        }
        else
        {
            ROS_WARN("Could not move arm to visual servo start position.");
            success = false;
        }
    }
    else
    {
        if(state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("Could not move arm to visual servo start position. Action preempted.");
            success = false;
        }
        else
        {
            ROS_INFO("Goal not reached. Unhandled action state: %s",state.toString().c_str());
            success = false;
        }
    }

    if(success)
    {
        bool first = true;      
        internal_servo_loop_running = true;
        while (node_handle.ok() && internal_servo_loop_running)
        {
          // Call servo client to initialize the servo node.
          std::string request_message;
          if(first)
          {
              traveled_servo_distance = 0.0;
              request_message = "initialize";
              visual_servo_control::request_servo_velocity_vector servo_velocity_vector_service;  
              servo_velocity_vector_service.request.request_message = request_message;               
              if (servo_velocity_vector_client.call(servo_velocity_vector_service))
              {
                  // If servo node is initialized, then start cycling. Otherwise keep trying. 
                  if(servo_velocity_vector_service.response.initialized)
                  {
                      first = false;
                  }
                  else
                  {
                      ROS_INFO("Retrying to initialize visual servo node.");
                  }
              }
              else
              {
                  ROS_INFO("Could not call service for initialization.");
              }
          }
          else
          {
              // Call image acquisition node's service to get an image.
              std::string trigger_message             = "trigger";
              image_acquisition::request_image          image_service;  
              image_service.request.request_message   = trigger_message;   
              sensor_msgs::Image                        ros_image; 
              if(image_client_rgb.call(image_service))
              {          
                  // Retrieve image.
                  ros_image                         = image_service.response.image;
                  most_recent_image_timestamp.data  = ros_image.header.stamp;

                  // Get fruit detection node's features from acquired image.
                  fruit_detection::request_features   features_service;  
                  features_service.request.image    = ros_image;
                  if(request_features_client.call(features_service))
                  {
                      // Get servo velocity vector with acquired features. 
                      request_message = "cycle";
                      visual_servo_control::request_servo_velocity_vector servo_velocity_vector_service;  
                      servo_velocity_vector_service.request.request_message = request_message;
                      servo_velocity_vector_service.request.pepper_blob_center_of_gravity_x = features_service.response.pepper_blob_center_of_gravity_x;
                      servo_velocity_vector_service.request.pepper_blob_center_of_gravity_y = features_service.response.pepper_blob_center_of_gravity_y;                                                 
                      if (servo_velocity_vector_client.call(servo_velocity_vector_service))
                      {                
                          // Get the error values.
                          std::vector<double> velocity_vector(servo_velocity_vector_service.response.servo_velocity_vector ); 
                          float error_x = servo_velocity_vector_service.response.error_x;
                          float error_y = servo_velocity_vector_service.response.error_y;

                          //ROS_INFO("Velocity vector has been recieved: %f %f %f %f %f %f", velocity_vector[0] , velocity_vector[1] , velocity_vector[2] , velocity_vector[3] , velocity_vector[4] , velocity_vector[5] );
                          //ROS_INFO("Error (x,y) recieved : %f %f", error_x , error_y);

                          // Check if the servo node found something useful or not. 
                          // Otherwise the arm cannot move in this cycle.
                          if(servo_velocity_vector_service.response.found_vector)
                          {           
                              // Adjust robot arm given the calculated error.
                              moveArm(velocity_vector, error_x, error_y, most_recent_image_timestamp);

                              // Check if fruit is reached. If so stop the servo node movement.
                              // Currently the loop is executed indefinetely.
                              /*
                              if(fruitReached())
                              {
                                  internal_servo_loop_running = false;
                                  external_servo_loop_running = false;

                                  std_msgs::Bool trigger;
                                  trigger.data = false;
                                  servo_startstop_publisher.publish(trigger); 
                              }
                              */
                          }
                          else
                          {
                              ROS_INFO("No velocity vector has been recieved:");
                          }
                      }
                      else
                      {
                          ROS_INFO("Could not call service for cycling the servo node.");
                      }
                  }
                  else
                  {
                      ROS_INFO("Could not call fruit detection feature service.");
                  }
              }
              else
              {
                   ROS_INFO("Could not call image detection service.");   
              }
        }
      }
      ros::spinOnce();
      ros::Rate rate(10.0);
      rate.sleep();
      transition = "goal reached";
    }
    else
    {
        transition = "goal not reached";
    }

    return transition;
}