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

#include "ScanPlantState.h"





/*
*   Service client function to request to endpoint state of the robot.
*   This information is needed to save the pose that corresponds to 
*   the view with the most recognized pepper.
*/
geometry_msgs::PoseStamped ScanPlantState::requestEndPointState()
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
void ScanPlantState::EndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
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
*   Requests an image from Image Aquisition node.
*   Requests blob size in that image from Fruit detection node.
*   If area is larger than any before, store it with current end-effector pose.
*/
bool ScanPlantState::detectFruit()
{
    // Request image from node that is responsible for image acquisition.
    HalconCpp::HImage                       halcon_image;
    std::string trigger_message           = "trigger";
    image_acquisition::request_image        image_service;  
    image_service.request.request_message = trigger_message;   

    if (image_client_rgb.call(image_service))
    {       
        // Get fruit detection node's features from acquired image.
        fruit_detection::request_features   features_service;  
        features_service.request.image    = image_service.response.image;

        if(request_features_client.call(features_service))
        {                                                          
            // Add fruit to global list.
            detected_fruit fruit;
            fruit.area = features_service.response.pepper_blob_size;
            fruit.pose = robot_endpoint_pose;
            // detected_fruits.push_back(fruit);

            ROS_INFO("Detected ripe fruit area : %i", fruit.area);
            
            // Check if detected pepper in current image is larger than in any other image.
            // If so, store it globally with its corresponding pose.
            if(fruit.area > best_fruit.area)
            {
                best_fruit.area = fruit.area;
                best_fruit.pose = robot_endpoint_pose;
            }
        }
        else
        {
            ROS_INFO("Could not call fruit detection feature service.");
        }            
    }
    else
    {
        ROS_INFO("Could not call image aqcuisition service.");
    }
}




std::string ScanPlantState::goToScanWaypoint(XmlRpc::XmlRpcValue waypoint)
{
    // Create the action client for arm movement. True causes the client to spin its own thread.
    actionlib::SimpleActionClient<robot_control::MoveArmAction> action_client_right("robot_control_right_arm", true); 

    ROS_INFO("Waiting for action server for moving rigth Baxter arm to start.");
    action_client_right.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server for moving rigth Baxter arm is started.");

    robot_control::MoveArmGoal goal_message;
    goal_message.goal_pose.header.stamp    = ros::Time::now();
    goal_message.goal_pose.header.frame_id = "base";
    goal_message.goal_pose.pose.position.x = waypoint[0];
    goal_message.goal_pose.pose.position.y = waypoint[1];
    goal_message.goal_pose.pose.position.z = waypoint[2];
    goal_message.goal_pose.pose.orientation.x = waypoint[3];
    goal_message.goal_pose.pose.orientation.y = waypoint[4];
    goal_message.goal_pose.pose.orientation.z = waypoint[5];
    goal_message.goal_pose.pose.orientation.w = waypoint[6];

    ROS_INFO("Sending goal for scan waypoint.");
    action_client_right.sendGoal(goal_message);
    
    // Loop while waiting for the goal to execute. 
    // During this loop, the scanning of the crop is performed.     
    actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::ACTIVE;
    while(state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE)
    {
        state = action_client_right.getState();
        detectFruit();       
    }

    std:string result;
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (action_client_right.getResult()->goal_reached)
        {   
            ROS_INFO("Arm reached position.");
            result = "goal reached";
        }
        else
        {
            ROS_WARN("Could not move arm to position.");
            result = "goal not reached";
        }
    }
    else
    {
        if(state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("Could not move arm to position. Action preempted.");
            result = "goal not reached";
        }
        else
        {
            ROS_INFO("Goal not reached. Unhandled action state: %s",state.toString().c_str());
            result = "error";
        }
    }
    return result;
}





/*
*   Set starting point for visual servo control in next state.
*   If a pepper was detected that is greater than a threshold, 
*   then use that as visual start pose. Otherwise use hardcoded pose
*   from launchfile.
*/
void ScanPlantState::setVisualServoStartingPose(std::map<std::string, boost::any> * data)
{

    ROS_INFO("Largest detected area: %i", best_fruit.area);
    if(best_fruit.area > 0)
    {
        ROS_INFO("Visual servo start pose from scanning.");

        // Use the best pose found pose during scanning.
        // No transformation is needed because the gripper 
        // pose is already in the /base frame.
        geometry_msgs::PoseStamped visual_servo_start_pose;

        visual_servo_start_pose.header.frame_id = "base";
        visual_servo_start_pose.pose.position.x = best_fruit.pose.pose.position.x;
        visual_servo_start_pose.pose.position.y = best_fruit.pose.pose.position.y;
        visual_servo_start_pose.pose.position.z = best_fruit.pose.pose.position.z;
        visual_servo_start_pose.pose.orientation.x = best_fruit.pose.pose.orientation.x;
        visual_servo_start_pose.pose.orientation.y = best_fruit.pose.pose.orientation.y;
        visual_servo_start_pose.pose.orientation.z = best_fruit.pose.pose.orientation.z;
        visual_servo_start_pose.pose.orientation.w = best_fruit.pose.pose.orientation.w;

        (*data)["visual_servo_start_pose"] = visual_servo_start_pose;
    }
    else
    {
        ROS_INFO("Visual servo start pose from launch file.");

        // Hardcode visual servo starting pose from launch file.
        XmlRpc::XmlRpcValue servo_start_pose; 
        node_handle.getParam("visual_servo_start_pose", servo_start_pose);
        geometry_msgs::PoseStamped visual_servo_start_pose;

        visual_servo_start_pose.header.frame_id = "base";
        visual_servo_start_pose.pose.position.x = servo_start_pose[0];
        visual_servo_start_pose.pose.position.y = servo_start_pose[1];
        visual_servo_start_pose.pose.position.z = servo_start_pose[2];
        visual_servo_start_pose.pose.orientation.x = servo_start_pose[3];
        visual_servo_start_pose.pose.orientation.y = servo_start_pose[4];
        visual_servo_start_pose.pose.orientation.z = servo_start_pose[5];
        visual_servo_start_pose.pose.orientation.w = servo_start_pose[6];

        (*data)["visual_servo_start_pose"] = visual_servo_start_pose;
    }   
}






ScanPlantState::ScanPlantState() 
{     
    // Initially the goal is not reached.
    transition = "goal not reached";
     
    // Get home position from launch file.
    node_handle.getParam("scan_waypoint_1", scan_waypoint_1);
    node_handle.getParam("scan_waypoint_2", scan_waypoint_2);
    node_handle.getParam("scan_waypoint_3", scan_waypoint_3);
    node_handle.getParam("scan_waypoint_4", scan_waypoint_4);
     
    // Client for requesting an RGB image.
    image_client_rgb  = node_handle.serviceClient<image_acquisition::request_image>("/image_acquisition_node/request_image_rgb");

    // Client for requesting detected fruit features from image.
    request_features_client = node_handle.serviceClient<fruit_detection::request_features>("/fruit_detection_node/request_features");

    // Client for requesting the endpoint state of the robot. Used to track which pose detects the most pepper.
    robot_endpointstate_client  = node_handle.serviceClient<robot_control::request_endpointstate>("/request_right_endpoint_state");

    // Callback for continously updating the endpoint.
    robot_endpoint_state_subscriber = node_handle.subscribe("/robot/limb/right/endpoint_state", 1000, &ScanPlantState::EndpointStateCallback_Baxter, this);
}






std::string ScanPlantState::execute(std::map<std::string, boost::any> * data)
{
    // Clear previously detected fruits.
    detected_fruits.clear();
    best_fruit.area = 0;

    while(true)
    {
        detectFruit();
    }

    // Go to all scan waypoints.
    //goToScanWaypoint(scan_waypoint_1);
    //goToScanWaypoint(scan_waypoint_2);
    //goToScanWaypoint(scan_waypoint_3);
    //goToScanWaypoint(scan_waypoint_4);

    // For the next state, it is required that starting point is defined.
    setVisualServoStartingPose(data);

    return "goal reached";
}
