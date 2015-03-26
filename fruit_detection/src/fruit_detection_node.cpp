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

#include "fruit_detection_node.h"


/***************************************************************************************************
 *
 *   Memory clearing function for ROS->OPENCV->Halcon bridge. Empty because memory handling is
 *   taken care by OpenCV and ROS.   
 *
 **************************************************************************************************/
void clearProc(void* ptr)
{
     return;
}




/***************************************************************************************************
 *
 *   Bridge: ROS Image -> OpenCV Image -> Halcon Image.
 *
 *   Stores a given ROS image in the global variable 'most_recent_image', to be used by Halcon procedures.
 *
 **************************************************************************************************/
HalconCpp::HImage rosImage2Halcon(sensor_msgs::Image &ros_image)
{    
     cv::Mat image_opencv;
     cv::Mat planes[3];

     try {
          image_opencv = cv_bridge::toCvCopy(ros_image, "")->image;

          split(image_opencv,planes);

          HalconCpp::HImage halcon_image;

          GenImage3Extern(&halcon_image,"byte",image_opencv.cols, image_opencv.rows,
                 (long)planes[0].data,
                 (long)planes[1].data,
                 (long)planes[2].data,
                 (long)clearProc);

          return halcon_image;
     }
     catch (cv_bridge::Exception& e) 
     {
          ROS_INFO("Exception while doing ROS to Halcon image bridge using OpenCV : %s", e.what());
     }
}




/***************************************************************************************************
 *
 *   Service for requesting pepper features in a provided image.
 *
 **************************************************************************************************/
bool getPepperFeatures(fruit_detection::request_features::Request &request, fruit_detection::request_features::Response &response)
{
     // Transform ROS image to Halcon image format.
     HalconCpp::HImage halcon_image = rosImage2Halcon(request.image);

     // Define the name of the .hdvp function that is run.
     std::string procedure_name = find_pepper_features_function;
     try
     {          
          // Initialize the function.
          ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputIconicParamObject("Image", halcon_image);

          proc_call.SetInputCtrlParamTuple("HueLowerThreshold"  , hue_lower_threshold);
          proc_call.SetInputCtrlParamTuple("HueUpperThreshold"  , hue_upper_threshold);

          // Execute and time the function.
          ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );
          
          // Get feature points and save globally.
          proc_call.GetOutputCtrlParamTuple("cog_x",&cog_x_tuple);
          proc_call.GetOutputCtrlParamTuple("cog_y",&cog_y_tuple);
          proc_call.GetOutputCtrlParamTuple("size" ,&size_tuple);

          response.pepper_blob_center_of_gravity_x = cog_x_tuple[0].D();
          response.pepper_blob_center_of_gravity_y = cog_y_tuple[0].D();
          response.pepper_blob_size                = size_tuple[0].D();

          ROS_INFO("Center of Gravity of Pepper in Image: (%0.0f,%0.0f), with size %0.f", cog_x_tuple[0].D(), cog_y_tuple[0].D(), size_tuple[0].D());
     }
     catch (HDevEngineCpp::HDevEngineException& exception)
     {
          ROS_INFO("Error category: %d : %s",     exception.Category(),exception.CategoryText());
          ROS_INFO("Error number: %d",            exception.HalconErrorCode());
          ROS_INFO("Procedure: %s",                    exception.ExecProcedureName());
          ROS_INFO("Line: %d : %s",                    exception.ProgLineNum(), exception.ProgLineName());
          return false;
     }
     return true;

}






/***************************************************************************************************
 *
 *   Initializes the halcon procedures before they can be executed.
 *
 **************************************************************************************************/
void intializeHalconProcedures()
{
          ROS_INFO("Initializing Halcon procedures.");
          hdevengine.SetHDevOperatorImpl(new WindowHandlingImplementation);
          hdevengine.SetProcedurePath(fruit_detection_halcon_external_procedures_path.c_str());
          ROS_INFO("External procedures path for servo nodes : %s", fruit_detection_halcon_external_procedures_path.c_str());
}





/***************************************************************************************************
 *
 *   Runs the .hdev script once if initilized with function initializeHalconScript();
 *   TODO: write access function to get results. See Halcon manual.
 *
 **************************************************************************************************/
bool runHalconScript()
{
     if(halcon_script_initialized)
     {
          ROS_INFO("Executing Halcon script (.hdev) at %s", fruit_detection_halcon_script_path.c_str());
          script_results = hdev_script.Execute();
          ROS_INFO("Halcon script has been executed, results are available");
          return true;
     }
     else
     {
          ROS_INFO("Halcon script not yet initialized. Not executing program at %s", fruit_detection_halcon_script_path.c_str());
          return false;
     }
}





/***************************************************************************************************
 *
 *   Initializes the .hdev script by loading it into the halcon engine.
 *
 *   Note that only one Halcon script per ROS node can be loaded, executed and accessed.
 *   If you require more scripts within one node, use individual procedures instead.
 *   The use of a script is added here for reference. The same functionality can be used
 *   by halcon procedures.
 *
 *   The script is not executed here, but by calling runHalconScript()
 *
 **************************************************************************************************/
void initializeHalconScript()
{
     ROS_INFO("Initializing .hdev Program");

     hdevengine.SetHDevOperatorImpl(new WindowHandlingImplementation);
     hdevengine.SetProcedurePath(fruit_detection_halcon_external_procedures_path.c_str());

     ROS_INFO("Script path (.hdev file)  : %s", fruit_detection_halcon_script_path.c_str());
     ROS_INFO("External procedures path  : %s", fruit_detection_halcon_external_procedures_path.c_str());

     try
     {
          hdev_script.LoadProgram(fruit_detection_halcon_script_path.c_str());
          halcon_script_initialized = true;
          ROS_INFO("Halcon program initialisation succesful.");
     }
     catch (HDevEngineCpp::HDevEngineException& exception)
     {
          ROS_INFO("Error category: %d : %s",     exception.Category(),exception.CategoryText());
          ROS_INFO("Error number: %d",            exception.HalconErrorCode());
          ROS_INFO("Procedure: %s",                    exception.ExecProcedureName());
          ROS_INFO("Line: %d : %s",                    exception.ProgLineNum(), exception.ProgLineName());
     }
}





/***************************************************************************************************
 *
 *  Initializes the ROS node.
 *
 *  The program path of the .hdev file that is to be executed for image acquisition is passed in 
 *  the launchfile (halcon_program_path). Similarly, if the script uses any external procedures, 
 *  a path (halcon_ext_proc_path) is also passed. The halcon script is automaticcaly initialized.
 *
 *
 **************************************************************************************************/
int main(int argc, char **argv)
{
     // Initialize this rosnode with an empty name string. This allow the node's name
     // to be set in the launchfile so that multiple copies of this node with different
     // names can be launched. Although you would probably not run more than 1 servo node, 
     // it is good practice to initialize the ros node's name in the launch file. 
     ros::init(argc, argv, "");
     
     // The nodehandle is initilized with a tilde ("~") The tilde refers to the node handle's 
     // namespace. This could be any namespace, which is roughly equivalent to a node name. 
     // However, the tilde refers to the current node's namespace. Why is this useful? In a 
     // roslaunch file, you can pass parameters to the system. There are "global" parameters 
     // that exist in the global, or default, namespace. These fall directly inside the <launch> 
     // tag. However, you can also put parameters under a <node> tag, and these will be in the 
     // local, or private, namespace of that node.
     ros::NodeHandle node_handle("~");
        
     // Get general path parameters for Halcon from ROS launch file.
     node_handle.getParam("fruit_detection_halcon_script_path",                 fruit_detection_halcon_script_path);
     node_handle.getParam("fruit_detection_halcon_external_procedures_path",    fruit_detection_halcon_external_procedures_path);
     
     // Get specific path parameters for Halcon functions that are loaded.
     node_handle.getParam("find_pepper_features_function",  find_pepper_features_function);     

     // Get parameters for the pepper detection
     node_handle.getParam("hue_lower_threshold", hue_lower_threshold);
     node_handle.getParam("hue_upper_threshold", hue_upper_threshold);

     // Advertise service for providing fruit features given an image. 
     pepper_features_output_service = node_handle.advertiseService("request_features", getPepperFeatures);
          
     // You can either initialize a full halcon script or initialize a set of procedures.
     // It is recomended you write everything in procedures so you can run subroutines without
     // any initialization overhead. 
     halcon_script_initialized = false;
     intializeHalconProcedures(); // OR initializeHalconScript();
             
     // The ROS node will now enter the following loop. 
     bool first = true;
     while (node_handle.ok())
     {          
          // Do one spin of the rosnode, e.g. checking and executing callbacks.
          // (http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
          ros::spinOnce();
     }
    return 0;
}
