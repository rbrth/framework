#ifndef SCANPLANTSTATE_H_
#define SCANPLANTSTATE_H_

//openCV FIRST, otherwise it does not compile. Do not ask why..
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

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
#include <vector>
#include <XmlRpcValue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_control/MoveArmAction.h>
#include "HalconCpp.h"
#include "HDevEngineCpp.h"

#include <image_acquisition/request_image.h>
#include <robot_control/request_endpointstate.h>
#include <fruit_detection/request_features.h>

class ScanPlantState: public State {

private:	
	// Subscribers, Publishers, Clients, Listeners.
     tf::TransformListener   transform_listener;
     ros::ServiceClient      image_client_rgb;
     ros::ServiceClient      robot_endpointstate_client;
     ros::ServiceClient      request_features_client;
     ros::Subscriber         robot_endpoint_state_subscriber;
     
     // Global variables
     ros::NodeHandle                                        node_handle;
     std::string                                            transition; 
     geometry_msgs::PoseStamped                             robot_endpoint_pose;
     bool                                                   first;
     XmlRpc::XmlRpcValue                                    scan_waypoint_1;  
     XmlRpc::XmlRpcValue                                    scan_waypoint_2;  
     XmlRpc::XmlRpcValue                                    scan_waypoint_3;  
     XmlRpc::XmlRpcValue                                    scan_waypoint_4;

     HDevEngineCpp::HDevEngine hdevengine;
     std::string state_machine_halcon_external_procedures_path;
     std::string find_area_pepper_function;
     std::string close_window_function;
     int hue_lower_threshold;
     int hue_upper_threshold;

     struct detected_fruit {
          int area;
          geometry_msgs::PoseStamped   pose;
     } ;

     std::vector<detected_fruit>   detected_fruits;
     detected_fruit                best_fruit;


public:

	ScanPlantState();
	std::string execute(std::map<std::string,boost::any> * data);
     geometry_msgs::PoseStamped requestEndPointState();
     void EndpointStateCallback_Baxter(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg);
     bool detectFruit();
     void closeWindow();
     std::string goToScanWaypoint(XmlRpc::XmlRpcValue waypoint);
     void setVisualServoStartingPose(std::map<std::string, boost::any> * data);

};

#endif /* SCANPLANTSTATE_H_ */
