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

#include "image_acquisition_node.h"


sensor_msgs::Image convertCOLORSPACEImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp)
{
          HalconCpp::HTuple type;
          HalconCpp::HTuple height_pointer;
          HalconCpp::HTuple width_pointer;
          HalconCpp::HTuple channel_pointer;

          GetImagePointer1(halcon_image,&channel_pointer,&type,&width_pointer,&height_pointer);

          uint8_t *uint8_pointer = reinterpret_cast<uint8_t*>((unsigned char*)channel_pointer[0].L());

          int height = atoi(height_pointer.ToString().Text());
          int width  = atoi(width_pointer.ToString().Text());

          sensor_msgs::Image output_image;

          output_image.header.stamp     = timestamp.data;
          output_image.height           = height;
          output_image.width            = width;
          output_image.encoding         = "mono8";
          output_image.is_bigendian     = false;
          output_image.step             = 1 * width;

          output_image.data.resize(width*height);
          memcpy(output_image.data.data(), uint8_pointer, width*height);

          /*
          for(int i=0; i<(width*height);i++)
          {
               output_image.data.push_back(uint8_pointer[i]);
          }
          */

          return output_image;
}



 

sensor_msgs::Image convertRGBImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp)
{
          HalconCpp::HTuple type;
          HalconCpp::HTuple height_pointer;
          HalconCpp::HTuple width_pointer;
          HalconCpp::HTuple channel_red_pointer;
          HalconCpp::HTuple channel_green_pointer;
          HalconCpp::HTuple channel_blue_pointer;

          halcon_image.GetImagePointer3(&channel_red_pointer,&channel_green_pointer,&channel_blue_pointer,&type,&width_pointer,&height_pointer);

          uint8_t *uint8_pointer_red    = reinterpret_cast<uint8_t*>((unsigned char*)channel_red_pointer[0].L());
          uint8_t *uint8_pointer_green  = reinterpret_cast<uint8_t*>((unsigned char*)channel_green_pointer[0].L());
          uint8_t *uint8_pointer_blue   = reinterpret_cast<uint8_t*>((unsigned char*)channel_blue_pointer[0].L());

          int height = atoi(height_pointer.ToString().Text());
          int width  = atoi(width_pointer.ToString().Text());

          sensor_msgs::Image output_image;

          output_image.header.stamp     = timestamp.data;
          output_image.height           = height;
          output_image.width            = width;
          output_image.encoding         = "rgb8";
          output_image.is_bigendian     = false;
          output_image.step             = 3 * width;


          for(int i=0; i<(width*height);i++)
          {
               output_image.data.push_back(uint8_pointer_red[i]);
               output_image.data.push_back(uint8_pointer_green[i]);
               output_image.data.push_back(uint8_pointer_blue[i]);
          }
          

          return output_image;
}





sensor_msgs::Image convertGRAYImage_Halcon2Ros(HalconCpp::HImage halcon_image, std_msgs::Time timestamp)
{
          HalconCpp::HTuple type;
          HalconCpp::HTuple height_pointer;
          HalconCpp::HTuple width_pointer;
          HalconCpp::HTuple channel_pointer;

          GetImagePointer1(halcon_image,&channel_pointer,&type,&width_pointer,&height_pointer);

          uint8_t *uint8_pointer = reinterpret_cast<uint8_t*>((unsigned char*)channel_pointer[0].L());

          int height = atoi(height_pointer.ToString().Text());
          int width  = atoi(width_pointer.ToString().Text());

          sensor_msgs::Image output_image;

          output_image.header.stamp     = timestamp.data;
          output_image.height           = height;
          output_image.width            = width;
          output_image.encoding         = "mono8";
          output_image.is_bigendian     = false;
          output_image.step             = 1 * width;

          output_image.data.resize(width*height);
          memcpy(output_image.data.data(), uint8_pointer, width*height);

          /*
          for(int i=0; i<(width*height);i++)
          {
               output_image.data.push_back(uint8_pointer[i]);
          }
          */

          return output_image;
}






void publishImage(HalconCpp::HImage image, std_msgs::Time timestamp, std::string type)
{
     if(type == "RGB")
     {
          if(rgb_image_taken)
          {
               //ROS_INFO("Published a RGB image.");
               rgb_image_publisher.publish(convertRGBImage_Halcon2Ros(image, timestamp));
          }
          else
          {
               ROS_INFO("Could not publish most recent ROS RGB image. Image was not yet taken.");
          }
     }
     else
     {
          if(type == "GRAY")
          {
               if(gray_image_taken)
               {
                    //ROS_INFO("Published a GRAY image.");
                    gray_image_publisher.publish(convertGRAYImage_Halcon2Ros(image, timestamp));
               }
               else
               {
               ROS_INFO("Could not publish most recent ROS GRAY image. Image was not yet taken.");
               }
          }
          else
          {
               if(type == "COLORSPACE")
               {
                    if(colorspace_image_taken)
                    {
                         //ROS_INFO("Published a COLORSPACE image.");
                         colorspace_image_publisher.publish(convertCOLORSPACEImage_Halcon2Ros(most_recent_image_halcon_colorspace, timestamp));
                    }
                    else
                    {
                         ROS_INFO("Could not publish most recent ROS COLORSPACE image. Image was not yet taken.");
                    }
               }
               else
               {
                     ROS_INFO("Could not publish most recent ROS GRAY image. Type unknown.");
               }
          }
     }
}





/***************************************************************************************************
 *
 *   Service that provides new COLORSPACE Halcon image upon request. Returns true if 
 *   image is taken/initialized.
 *        
 **************************************************************************************************/
bool externalTrigger_RectifiedCOLORSPACEImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res)
{
          HalconCpp::HImage   image;
          std_msgs::Time      timestamp;

          grabRectifiedImageCOLORSPACE(image, timestamp);

          res.image = convertCOLORSPACEImage_Halcon2Ros(image, timestamp);
          
          if(req.request_message == "publish")
          {
               publishImage(most_recent_image_halcon_colorspace, timestamp, "COLORSPACE");
          }

          return true;
}







/***************************************************************************************************
 *
 *   Service that provides new RGB Halcon image upon request. Returns true if 
 *   image is taken/initialized.
 *        
 **************************************************************************************************/
bool externalTrigger_RectifiedRGBImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res)
{
          HalconCpp::HImage   image;
          std_msgs::Time      timestamp;

          grabRectifiedImageRGB(image, timestamp);

          res.image = convertRGBImage_Halcon2Ros(image, timestamp);
          
          if(req.request_message == "publish")
          {
               publishImage(image, timestamp, "RGB");
          }

          return true;
}






/***************************************************************************************************
 *
 *   Service that provides new Gray Halcon image upon request. Returns true if 
 *   image is taken/initialized.
 *        
 **************************************************************************************************/
bool externalTrigger_RectifiedGRAYImage(image_acquisition::request_image::Request &req, image_acquisition::request_image::Response &res)
{
          HalconCpp::HImage   image;
          std_msgs::Time      timestamp;

          grabRectifiedImageGRAY(image, timestamp);

          res.image = convertGRAYImage_Halcon2Ros(image, timestamp);

          if(req.request_message == "publish")
          {
               publishImage(image, timestamp, "GRAY");
          }

          return true;
}





/***************************************************************************************************
 *
 *   Service that provides camera parameters upon request. Returns true if camera parameters are
 *   already loaded, false otherwise.
 *        
 **************************************************************************************************/
bool getCameraParameters(image_acquisition::request_camera_parameters::Request &req, image_acquisition::request_camera_parameters::Response &res)
{
     if(camera_parameters_loaded)
     {
          res.f   =  camera_parameters[0].D();
          res.k1  =  camera_parameters[1].D();
          res.k2  =  camera_parameters[2].D();
          res.k3  =  camera_parameters[3].D();
          res.P1  =  camera_parameters[4].D();
          res.P2  =  camera_parameters[5].D();
          res.Sx  =  camera_parameters[6].D();
          res.Sy  =  camera_parameters[7].D();
          res.Cx  =  camera_parameters[8].D();
          res.Cy  =  camera_parameters[9].D();
          res.Iw  =  camera_parameters[10].D();
          res.Ih  =  camera_parameters[11].D();
          return true;
     }
     else
     {
          return false;
     }
}





/***************************************************************************************************
 *
 *	Initializes the camera by calling the corresponding hdvp function. 
 *   The function is initialized here and has the following inputs and output:
 *
 *   Input
 *        - Exposure
 *        - Gain
 *        - GainR
 *        - GainG
 *        - GainB
 *
 *        - CameraParametersPath : path to the .cam file holding the camera parameters, defined
 *                                 in launch file and loaded in global parameter camera_parameters_path
 *        
 *        
 *   Output
 *        - AcqHandle : handle for image acquisition, to save for global use.
 *
 *        - CamParam : camera parameters read from file specified in launchfile.
 *                     More information about paremeters in the array, can be found
 *                     here: http://www.halcon.com/download/reference/camera_calibration.html
 *
 *                         [0]  F       Focal length.
 *                         [1]  K1
 *                         [2]  K2
 *                         [3]  K3
 *                         [4]  P1
 *                         [5]  P2
 *                         [6]  Sx      Horizontal distance between two neighboring pixel cells.
 *                         [7]  Sy      Horizontal distance between two neighboring pixel cells.
 *                         [8]  Cx      X-Coordinate of image center.
 *                         [9]  Cy      Y-Coordinate of image center.
 *                         [10] Iw      Image Width.
 *                         [11] Ih      Image Height.
 *
 **************************************************************************************************/
void initializeCamera()
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = initialize_camera_function;
     try
     {          
          // Initialize the function.
          ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);

          // Define control parameters.
          HalconCpp::HTuple Exposure    =  exposure;
          HalconCpp::HTuple Gain        =  gain;
          HalconCpp::HTuple GainR       =  gainR;
          HalconCpp::HTuple GainG       =  gainG;
          HalconCpp::HTuple GainB       =  gainB;
          
          HalconCpp::HTuple CameraParametersPath = camera_parameters_path.c_str();
                  
          // Set control parameters in .hdvp function.
          proc_call.SetInputCtrlParamTuple("Exposure"  , Exposure);
          proc_call.SetInputCtrlParamTuple("Gain"      , Gain);
          proc_call.SetInputCtrlParamTuple("GainR"     , GainR);
          proc_call.SetInputCtrlParamTuple("GainG"     , GainG);
          proc_call.SetInputCtrlParamTuple("GainB"     , GainB);
          
          proc_call.SetInputCtrlParamTuple("CameraParametersPath", CameraParametersPath);
     
          //Execute and time the function.
          ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );
          
          // Get aqcuisition handle and save in global variable.
          proc_call.GetOutputCtrlParamTuple("AcqHandle", &camera_acquisition_handle);
          ROS_INFO("AcqHandle # : %ld",camera_acquisition_handle[0].L());
          
          // Get camera parameters and save in global variable.
          proc_call.GetOutputCtrlParamTuple("CamParam", &camera_parameters);
          camera_parameters_loaded = true;
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
 *   Grabs the next colorspace image using a .hdvp function
 *
 *   Input
 *        - AcqHandle : handle for image acquisition, use the global camera_acquisition_handle.
 *
 *        - CamParam : camera parameters. 
 *        
 *   Output
 *        - ImageRectified : resized and rectified grabbed image. 
 *
 **************************************************************************************************/
void grabRectifiedImageCOLORSPACE(HalconCpp::HImage &image, std_msgs::Time &timestamp)
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = grab_rectified_colorspace_image_function;
     try
     {          
          // Initialize the function.
          //ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputCtrlParamTuple("CamParam", camera_parameters);
          proc_call.SetInputCtrlParamTuple("AcqHandle", camera_acquisition_handle);
   
          //Execute and time the function.
          //ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          //ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );
          
          // Make sure the time of image aqcuisition is more or less known.
          most_recent_image_halcon_colorspace_timestamp.data = ros::Time::now();
          timestamp.data = ros::Time::now();
          
          // Get image and camera parameters and save in global variable.
          most_recent_image_halcon_colorspace = proc_call.GetOutputIconicParamObject("Image");
          image = proc_call.GetOutputIconicParamObject("Image");

          colorspace_image_taken = true;
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
 *	Grabs the next image using a .hdvp function
 *
 *   Input
 *        - AcqHandle : handle for image acquisition, use the global camera_acquisition_handle.
 *
 *        - CamParam : camera parameters. 
 *        
 *   Output
 *        - ImageRectified : resized and rectified grabbed image. 
 *
 **************************************************************************************************/
void grabRectifiedImageRGB(HalconCpp::HImage &image, std_msgs::Time &timestamp)
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = grab_rectified_rgb_image_function;
     try
     {          
          // Initialize the function.
          //ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputCtrlParamTuple("CamParam", camera_parameters);
          proc_call.SetInputCtrlParamTuple("AcqHandle", camera_acquisition_handle);

          //Execute and time the function.
          //ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          //ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );

          // Make sure the time of image aqcuisition is more or less known.
          //most_recent_image_halcon_rgb_timestamp.data = ros::Time::now();
          timestamp.data = ros::Time::now();
          
          // Get image and camera parameters and save in global variable.
          //most_recent_image_halcon_rgb = proc_call.GetOutputIconicParamObject("ImageRectified");
          image = proc_call.GetOutputIconicParamObject("ImageRectified");

          rgb_image_taken              = true;
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
 *   Grabs the next image using a .hdvp function
 *
 *   Input
 *        - AcqHandle : handle for image acquisition, use the global camera_acquisition_handle.
 *
 *        - CamParam : camera parameters. 
 *        
 *   Output
 *        - ImageRectified : resized and rectified grabbed image. 
 *
 **************************************************************************************************/
void grabRectifiedImageGRAY(HalconCpp::HImage &image, std_msgs::Time &timestamp)
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = grab_rectified_gray_image_function;
     try
     {    
          double begin_time = ros::Time::now().toNSec();      
          // Initialize the function.
          //ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputCtrlParamTuple("CamParam", camera_parameters);
          proc_call.SetInputCtrlParamTuple("AcqHandle", camera_acquisition_handle);
   
          //Execute and time the function.
          //ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          //ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );

          // Make sure the time of image aqcuisition is more or less known.
          //most_recent_image_halcon_gray_timestamp.data = ros::Time::now();
          timestamp.data = ros::Time::now();
          
          // Get image and camera parameters and save in global variable.
          //most_recent_image_halcon_gray = proc_call.GetOutputIconicParamObject("ImageRectifiedGray");
          image = proc_call.GetOutputIconicParamObject("ImageRectifiedGray");

          gray_image_taken = true;
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
 *	Displays the last image that is stored globally using a .hdvp function.
 *
 *   Input
 *        - Image : resized and rectified grabbed image to display. 
 *
 **************************************************************************************************/
void displayGrabbedImage(HImage image)
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = display_grabbed_image;
     try
     {          
          // Initialize the function.
          ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputIconicParamObject("Image", image);
   
          // Execute and time the function.
          ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );
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
 *	Closes the framegrabber of the camera.
 *
 *   Input
 *        - AcqHandle : handle for image acquisition, use the global camera_acquisition_handle.
 *        
 **************************************************************************************************/
void closeFrameGrabber()
{
     // Defines the name of the .hdvp function that is run.
     std::string procedure_name = close_frame_grabber_function;
     try
     {          
          // Initialize the function.
          ROS_INFO("Initializing HDev function %s.hdvp", procedure_name.c_str());
          HDevEngineCpp::HDevProcedure proc(procedure_name.c_str());
          HDevEngineCpp::HDevProcedureCall proc_call(proc);
        
          // Set control parameters in .hdvp function.
          proc_call.SetInputCtrlParamTuple("AcqHandle", camera_acquisition_handle);
   
          //Execute and time the function.
          ROS_INFO("Executing HDev function %s.hdvp", procedure_name.c_str());
          double begin_time = ros::Time::now().toNSec();
          proc_call.Execute();
          double end_time   = ros::Time::now().toNSec();
          ROS_INFO("Succesfully executed HDev function %s.hdvp", procedure_name.c_str());
          ROS_INFO("Execution time : %.3f seconds.", (end_time-begin_time)/1000000000 );
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
 *   Initializes the halcon procedures before they can be executed.
 *
 **************************************************************************************************/
void intializeHalconProcedures()
{
       	ROS_INFO("Initializing Halcon procedures.");
	     hdevengine.SetHDevOperatorImpl(new WindowHandlingImplementation);
	     hdevengine.SetProcedurePath(image_acquisition_halcon_external_procedures_path.c_str());
	     ROS_INFO("External procedures path for image acquisition: %s", image_acquisition_halcon_external_procedures_path.c_str());
}





/***************************************************************************************************
 *
 *   Runs the .hdev script once if initilized with function initializeHalconScript();
 *   TODO: write access function to get results.
 *
 **************************************************************************************************/
bool runHalconScript()
{
	if(initialized)
	{
		ROS_INFO("Executing Halcon script (.hdev) at %s", image_acquisition_halcon_script_path.c_str());
		script_results = hdev_script.Execute();
		ROS_INFO("Halcon script has been executed, results are available");
          return true;
	}
	else
	{
		ROS_INFO("Halcon script not yet initialized. Not executing program at %s", image_acquisition_halcon_script_path.c_str());
		return false;
	}
}





/***************************************************************************************************
 *
 *	Initializes the .hdev script by loading it into the halcon engine.
 *
 *	The program is not executed here.
 *
 **************************************************************************************************/
void initializeHalconScript()
{
	ROS_INFO("Initializing .hdev Program");

	hdevengine.SetHDevOperatorImpl(new WindowHandlingImplementation);
	hdevengine.SetProcedurePath(image_acquisition_halcon_external_procedures_path.c_str());

	ROS_INFO("Script path (.hdev file)  : %s", image_acquisition_halcon_script_path.c_str());
	ROS_INFO("External procedures path  : %s", image_acquisition_halcon_external_procedures_path.c_str());

	try
	{
		hdev_script.LoadProgram(image_acquisition_halcon_script_path.c_str());
		initialized = true;
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
 *	Initializes the ROS node.
 *
 *	The program path of the .hdev file that is to be executed for image acquisition is passed in 
 *	the launchfile (halcon_program_path). Similarly, if the script uses any external procedures, 
 *	a path (halcon_ext_proc_path) is also passed. The halcon script is automaticcaly initialized.
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
    	
    	// Parameters that are initialized at start of the ros node.
    	// TODO: these should be made specific to a function.
	rgb_image_taken          = false;
     gray_image_taken         = false;
     colorspace_image_taken   = false;
	camera_parameters_loaded = false;
     initialized              = false;
     global_lock              = false;

     // Get general path parameters for Halcon from ROS launch file.
     node_handle.getParam("image_acquisition_halcon_script_path",              image_acquisition_halcon_script_path);
    	node_handle.getParam("image_acquisition_halcon_external_procedures_path", image_acquisition_halcon_external_procedures_path);
    	node_handle.getParam("camera_parameters_path",         camera_parameters_path);
    	
    	// Get specific path parameters for Halcon functions that are loaded.
    	node_handle.getParam("initialize_camera_function",               initialize_camera_function);
    	node_handle.getParam("grab_rectified_rgb_image_function",        grab_rectified_rgb_image_function);
     node_handle.getParam("grab_rectified_gray_image_function",       grab_rectified_gray_image_function);
    	node_handle.getParam("grab_rectified_colorspace_image_function", grab_rectified_colorspace_image_function);
     node_handle.getParam("display_grabbed_image",                    display_grabbed_image);
    	node_handle.getParam("close_frame_grabber_function",             close_frame_grabber_function);

     // Get camera initialization parameters.
     node_handle.getParam("exposure", exposure);
     node_handle.getParam("gain" ,    gain);
     node_handle.getParam("gainR",    gainR);
     node_handle.getParam("gainG",    gainG);
     node_handle.getParam("gainB",    gainB);
    		
	// You can either initialize a full halcon script or initialize a set of procedures.
	// It is recomended you write everything in procedures so you can run subroutines without
	// any initialization overhead. 
	intializeHalconProcedures(); // OR initializeHalconScript();
	
	// Initialize camera to open frame grabber and get camera parameters.	     
	initializeCamera();

     // Service for providing camera parameters.
     camera_parameters_service = node_handle.advertiseService("request_camera_parameters", getCameraParameters); 

     // Service for providing Halcon's HImages in ROS image format.
     image_service_rgb        = node_handle.advertiseService("request_image_rgb",  externalTrigger_RectifiedRGBImage);
     image_service_gray       = node_handle.advertiseService("request_image_gray", externalTrigger_RectifiedGRAYImage);
     image_service_colorspace = node_handle.advertiseService("request_image_colorspace", externalTrigger_RectifiedCOLORSPACEImage);

     // Publisher for ROS images.
     rgb_image_publisher           = node_handle.advertise<sensor_msgs::Image>("rgb_image", 1);
     gray_image_publisher          = node_handle.advertise<sensor_msgs::Image>("gray_image", 1);
     colorspace_image_publisher    = node_handle.advertise<sensor_msgs::Image>("colorspace_image", 1);
             
     // The ROS node will now enter the following loop.	
     bool first = true;
     while (node_handle.ok())
     {
          
               // Grab new image.
               //HalconCpp::HImage   image;
               //std_msgs::Time      timestamp;
               //grabRectifiedImageGRAY(image, timestamp);
              
               // Publish the image.
               //publishImage(image, timestamp, "GRAY");
               // displayGrabbedImage(image);
               
               // Make sure no other image is grabbed withing 1 exposure time.
               // Otherwise you may end up with scrambled images.
               // The driver's timeout is not yet working.
               ros::Duration(exposure/1000).sleep();
          
          // Do one spin of the rosnode, e.g. checking and executing callbacks.
          // (http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
          ros::spinOnce();
     }
	return 0;
}
