#ifndef VISUALSERVOSTATE_H_
#define VISUALSERVOSTATE_H_

#include "State.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Bool.h"

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

#include <visual_servo_control/request_servo_velocity_vector.h>
#include <fruit_detection/request_features.h>
#include <robot_control/request_endpointstate.h>
#include <image_acquisition/request_image.h>


class VisualServoState: public State {

private:	

	// Subscribers, Publishers, Clients, Listeners.
    tf::TransformListener          transform_listener;
    ros::ServiceClient             robot_endpointstate_client;
    ros::ServiceClient             servo_velocity_vector_client;
    ros::ServiceClient             request_features_client;
    
    ros::ServiceClient             image_client_rgb;
    ros::ServiceClient             image_client_gray;
    ros::Subscriber                robot_endpoint_state_subscriber;
    ros::Publisher                 cancel_robot_movement_publisher;
    ros::Publisher                 servo_setgoal_publisher;
    ros::Publisher                 servo_startstop_publisher;
    ros::Publisher                 servo_execute_publisher;

    // Global variables.
    ros::NodeHandle                node_handle;
    geometry_msgs::PoseStamped     robot_endpoint_pose;
    std::string                    transition;
    double                         traveled_servo_distance;
    bool                           external_servo_loop_running;
    bool                           internal_servo_loop_running;
    double                         forward_distance_step;
    std_msgs::Time                 most_recent_image_timestamp;


public:
	
	VisualServoState();
	std::string execute(std::map<std::string,boost::any> * data);
    void EndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
    void moveArm(std::vector<double> velocity_vector, float error_x, float error_y, std_msgs::Time image_time);
    geometry_msgs::PoseStamped requestEndPointState();
};

#endif /* VISUALSERVOSTATE_H_ */
