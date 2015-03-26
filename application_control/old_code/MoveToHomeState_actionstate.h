#ifndef MOVETOHOMESTATE_H_
#define MOVETOHOMESTATE_H_

#include <SimpleActionState.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "baxter_core_msgs/EndpointState.h"
#include "baxter_core_msgs/SolvePositionIK.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Quaternion.h" 
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>
#include "control_msgs/FollowJointTrajectoryAction.h"



class MoveToHomeState: public SimpleActionState<control_msgs::FollowJointTrajectoryAction, control_msgs::FollowJointTrajectoryGoal>
{
private:
	ros::NodeHandle node_handle;
	std::string transition;

public:
     bool                                    check_cartesian_endpoints;
     bool                                    first_state_visit;
     ros::Subscriber                         robot_left_endpoint_state_subscriber;
     ros::Subscriber                         robot_right_endpoint_state_subscriber;
     tf::TransformListener                   transform_listener;
     std::vector<sensor_msgs::JointState>    joint_states;
     geometry_msgs::PoseStamped              goal_pose;
     
     void robotRightEndpointStateCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
     void robotLeftEndpointStateCallback(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
     bool getJointPositions(std::string side, geometry_msgs::PoseStamped pose);
	MoveToHomeState();
	std::string execute(std::map<std::string,boost::any> * data);
};

#endif /* MOVETOHOMESTATE_H_ */
