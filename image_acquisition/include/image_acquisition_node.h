#ifndef IMAGE_ACQUISITION_NODE_H
#define IMAGE_ACQUISITION_NODE_H


//openCV FIRST, otherwise it does not compile. Do not ask why..
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "sensor_msgs/Image.h"
#include <vector>
#include <stdint.h>
#include <string.h>

#include "HalconCpp.h"
#include "HDevEngineCpp.h"
#include "window_handling.h"

#include <image_acquisition/request_camera_parameters.h>
#include <image_acquisition/request_image.h>


/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
              The global variables in this frame are used in the HDevEngine.     
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
HDevEngineCpp::HDevEngine hdevengine;
HDevEngineCpp::HDevProgram hdev_script;
HDevEngineCpp::HDevProgramCall script_results;

std::string image_acquisition_halcon_script_path;
std::string image_acquisition_halcon_external_procedures_path;
std::string camera_parameters_path;

std::string initialize_camera_function;
std::string grab_rectified_rgb_image_function;
std::string grab_rectified_gray_image_function;
std::string grab_rectified_colorspace_image_function;
std::string display_grabbed_image;
std::string close_frame_grabber_function;

HalconCpp::HTuple camera_parameters;
bool camera_parameters_loaded;

bool global_lock;
bool initialized;

HalconCpp::HTuple   camera_acquisition_handle;

HalconCpp::HImage   most_recent_image_halcon_rgb;
std_msgs::Time      most_recent_image_halcon_rgb_timestamp;
bool rgb_image_taken;

HalconCpp::HImage   most_recent_image_halcon_gray;
std_msgs::Time      most_recent_image_halcon_gray_timestamp;
bool gray_image_taken;

HalconCpp::HImage   most_recent_image_halcon_colorspace;
std_msgs::Time      most_recent_image_halcon_colorspace_timestamp;
bool colorspace_image_taken;



sensor_msgs::Image  most_recent_image_ros_rgb;
std_msgs::Time      most_recent_image_ros_rgb_timestamp;

int exposure;
int gain;
int gainR;
int gainG;
int gainB;

int hue_lower_threshold;
int hue_upper_threshold;
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/







/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                          Global publishers and Subscribers of the ROS node   
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
ros::Publisher      rgb_image_publisher;
ros::Publisher      gray_image_publisher;
ros::Publisher      colorspace_image_publisher;
ros::ServiceServer  camera_parameters_service;
ros::ServiceServer  image_service_rgb;
ros::ServiceServer  image_service_gray;
ros::ServiceServer  image_service_colorspace;
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/





/*°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
                              All functions within this ros node.
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°*/
sensor_msgs::Image convertRGBImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp);
sensor_msgs::Image convertGRAYImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp);
sensor_msgs::Image convertCOLORSPACEImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp);

void publishImage(HalconCpp::HImage image, std::string type);

bool externalTrigger_RectifiedRGBImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res);
bool externalTrigger_RectifiedGRAYImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res);
bool externalTrigger_RectifiedCOLORSPACEImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res);

bool getCameraParameters(image_acquisition::request_camera_parameters::Request &req, image_acquisition::request_camera_parameters::Response &res);

void grabRectifiedImageRGB(HalconCpp::HImage &halcon_image, std_msgs::Time &timestamp);
void grabRectifiedImageGRAY(HalconCpp::HImage &halcon_image, std_msgs::Time &timestamp);
void grabRectifiedImageCOLORSPACE(HalconCpp::HImage &halcon_image, std_msgs::Time &timestamp);


void initializeCamera();
void grabRectifiedImage();
void displayGrabbedImage();
void closeFrameGrabber();

void initializeHalconScripts();

int main(int argc, char **argv);
/*‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡‡*/



#endif
