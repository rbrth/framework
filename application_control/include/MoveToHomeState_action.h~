#ifndef MOVETOHOMESTATE_H_
#define MOVETOHOMESTATE_H_

#include "State.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "baxter_core_msgs/EndpointState.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/JointCommand.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Quaternion.h" 
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>
#include <math.h> 
#include <XmlRpcValue.h>


class MoveToHomeState: public State {

private:	
	// Subscribers, Publishers, Clients, Listeners.
	ros::Subscriber                         robot_left_endpoint_state_subscriber;
     ros::Subscriber                         robot_right_endpoint_state_subscriber;
     ros::Publisher                          robot_left_joint_command_publisher;
     ros::Publisher                          robot_right_joint_command_publisher;
     ros::ServiceClient                      inverse_kinematics_client;
     tf::TransformListener                   transform_listener;
     
     // Global variables
     ros::NodeHandle                         node_handle;
     std::string                             transition;
     bool                                    check_cartesian_endpoints;
     geometry_msgs::Pose                     robot_left_endpoint_pose;
     geometry_msgs::Pose                     robot_right_endpoint_pose; 
     XmlRpc::XmlRpcValue                     home_position;   
      
public:

     void LeftEndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
     void RightEndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
     bool calculateJointPositions_Baxter(std::string side, geometry_msgs::PoseStamped pose, std::vector<sensor_msgs::JointState> &joint_states);
	void MoveToHome_Baxter();
	
	MoveToHomeState();
	std::string execute(std::map<std::string,boost::any> * data);
};

#endif /* MOVETOHOMESTATE_H_ */
