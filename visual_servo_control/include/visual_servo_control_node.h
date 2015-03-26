#ifndef visual_servo_control_node_H
#define visual_servo_control_node_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "sensor_msgs/Image.h"
#include <vector>
#include <stdint.h>
#include <string.h>

#include <visual_servo_control/request_servo_velocity_vector.h>
#include <image_acquisition/request_camera_parameters.h>

#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>
#include <visp/vpConfig.h>
#include <visp/vpDebug.h> 
#include <stdlib.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpRobotAfma4.h>
#include <visp/vpIoTools.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpDot.h>


/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                                        Camera parameters 
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
bool camera_parameters_loaded;

double f;
double k1;
double k2;
double k3;
double P1;
double P2;
double Sx;
double Sy;
double Cx;
double Cy;
double Iw;
double Ih;

double fxi;
double fyi;
double cxi;
double cyi;
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/





/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                        The global variables for the servo simulation   
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
bool initialized;
bool simulation_initialized;
bool servo_initialized;
bool program_lock;

// Here we define the desired and initial/current position of the camera as two homogeneous matrices
// Initial is not allowed to be all zero.

// Desired pose camera relative to object. In other words, target pose of camera relative to object.
vpHomogeneousMatrix cdMo(0, 0, 0.01, 0, 0, 0); 

// Actual pose camera relative object. In other words, position of object in camera frame is known.
vpHomogeneousMatrix cMo(0.15, -0.1, 1,vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

// For the simulation we need to create two homogeneous transformations wMc and wMo, 
// respectively to define the position of the camera, and the position of the object in 
// the world frame.
 
// Position of camera in world frame (get from robot).
vpHomogeneousMatrix wMc;

// Position of the object in world frame. 
vpHomogeneousMatrix wMo;

vpPoint point[4];
vpServo task;
vpFeaturePoint p[4], pd[4];
vpFeaturePoint p_cog, pd_cog;
int iteration;
                            
// We create an instance of a free flying camera. 
vpSimulatorCamera robot;

// Current position and orientation of simulated camera.
tf::Vector3    simulated_camera_position;
tf::Quaternion simulated_camera_rotation;
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/





/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                          Global publishers and Subscribers of the ROS node   
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
ros::ServiceServer      velocity_vector_output_service;
ros::ServiceClient      camera_parameters_client;
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/




/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                              All functions within this ros node.
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
bool getVelocityVector_CalTab(visual_servo_control::request_servo_velocity_vector::Request &req, visual_servo_control::request_servo_velocity_vector::Response &res);
bool simulateVelocityVectorCalculation(visual_servo_control::request_servo_velocity_vector::Request &req, visual_servo_control::request_servo_velocity_vector::Response &res);
int main(int argc, char **argv);
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/
#endif
