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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_control/MoveArmAction.h>

class MoveToHomeState: public State {

private:	
	// Subscribers, Publishers, Clients, Listeners.
     tf::TransformListener    transform_listener;
     
     // Global variables
     ros::NodeHandle          node_handle;
     std::string              transition;
     XmlRpc::XmlRpcValue      home_position;   
     bool                     first;

public:
	void MoveToHome_Baxter();
	
	MoveToHomeState();
	std::string execute(std::map<std::string,boost::any> * data);
};

#endif /* MOVETOHOMESTATE_H_ */
