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


                                                             /* 
                            		                         ### 
                         		                       ## ##    
                         		                      ##  ##    
                         		                       ####     
                         		                         : 
                         		                       #####   
                         		                       ######   
                         		                       ##  ##   
                         		                       ##  ##   
                         		                       ##  ##   
                         		                       ##  ##########     
                             		                       ##  #############  
                         		                  #######  ###############     
                         		              #############################    
                         		        .###################################   
                         		       #####################################;  
                         		       ##                                 ##.  
                         		       ##                                 ##   
                         		       #####################################   
                         		       ##                                 ##   
                         		       ##       The cake is a lie         ##   
                         		       ##                                 ###  
                         		    #####                                 #####     
                         		   ### ##################################### ###    
                         		  ###  ##                                 ##  ###   
                         		  ##   ##                                 ##   ##   
                         		  ##   ##,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,##   ##   
                         		   ##  #####################################  ##    
                         		    ##                                       ##     
                         		     ####                                 #### 
                         		       ######                         ######   
                                             ##############################*/
         			          
          		

#include "visual_servo_control_node.h"


bool getVelocityVector_Point(visual_servo_control::request_servo_velocity_vector::Request &req, visual_servo_control::request_servo_velocity_vector::Response &res)
{
	// This part of the function gets locked if it is still calculating the next velocity
	// vector in the servo application. Hence, this service will return false if it is still busy. 
	ROS_INFO("Service called (getVelocityVector_Point)");
	std::string request_message = req.request_message.c_str();
	if(!program_lock)
	{
	     ROS_INFO("Program not locked, starting getVelocityVector_Point");
		program_lock = true;
          try {
               // If the servo system is not yet initialized, then initialize global variables.
               // The control message is compared to the incoming request message.
               std::string control_message = "initialize";
               if(!servo_initialized && camera_parameters_loaded && request_message == control_message)
               {       
                    ROS_INFO("Initializing servo. Camera are parameters loaded. Service request message was : %s", control_message.c_str());

                    int cog_x = req.pepper_blob_center_of_gravity_x;
                    int cog_y = req.pepper_blob_center_of_gravity_y;

                         // Check if found center of gravity is found.
                         if(cog_x > -1 && cog_y > -1)
                         {
                              // Create point from found coordinates in image.
                              vpImagePoint cog_image_coordinate = vpImagePoint (cog_x, cog_y);
            
                              // Set intrinsic camera parameters, since we will have to convert image pixel 
                              // coordinates to visual features expressed in meters.
                              // vpCameraParameters(px,py,u0,v0) where:
                              //   -px = horizontal pixel size. 
                              //   -py = vertical pixel size.
                              //   -u0 = X-Coordinate of image center.
                              //   -v0 = Y-Coordinate of image center.
                              // px and py are the ratios between the focal and the size of the pixel on the CCD
                              // so a let say a 6mm lens with a 10micrometers pixel size gi ves you px = 6e-3/1e-5 = 600
                              // In our case (F/Sx) : 0.00546213886287705 / 1.09363999535642e-05 = 500.
                              vpCameraParameters camera_params(500, 500, 240, 320);
                              //ROS_INFO("Real image center coordinates in image (x,y) : %f %f", camera_parameters[8].D(), camera_parameters[9].D() );

                              // Create a vpFeaturePoint using a vpFeaturePoint and the parameters of the camera. 
                              // The vpDot contains only the pixel coordinates of the point in an image. 
                              // Thus this method uses the camera parameters to compute the meter coordinates 
                              // in x and y in the image plane.
                              vpFeatureBuilder::create(p_cog, camera_params, cog_image_coordinate);   

                              // It is not possible to compute the depth of the point Z (meter) in the camera 
                              // frame in the vpFeatureBuilder above.  
                              // This coordinate is needed in vpFeaturePoint to compute the interaction matrix. 
                              p_cog.set_Z(0.40);  

                              // Sets the desired position (x,y,z) of the visual feature.
                              pd_cog.buildFrom(0,0,0.40);

                              // Define the task: eye-in-hand. 
                              // The robot is hence controlled in the camera frame.
                              task.setServo(vpServo::EYEINHAND_CAMERA);

                              // The result should be a point on a point.
                              task.addFeature(p_cog,pd_cog);

                              // Set the gain of the servo task: rate of change.
                              task.setLambda(0.99);

                              // Display task information.
                              task.print();

                              // The servo loop will start at iteration zero.
                              iteration = 0;
                              servo_initialized = true;
                              res.initialized = true;
                              res.found_vector = false;
                              ROS_INFO("Servo initialized.");
                         }
                         else
                         {
                              ROS_INFO("Servo not initialized : center of gravity not found in image.");                         
                              servo_initialized = false;
                              res.initialized = false;
                              res.found_vector = false;
                         }
               }  
               else
               {
                    // When initialized, perform servo loop.
                    std::string control_message = "cycle";
                    if(servo_initialized && request_message == control_message)
                    {
                         ROS_INFO("Performing servo loop iteration : %d", iteration);

                         int cog_x = req.pepper_blob_center_of_gravity_x;
                         int cog_y = req.pepper_blob_center_of_gravity_y;
                      
                              // Check if found center of gravity is found.
                              if(cog_x > -1 && cog_y > -1)
                              {
                                   // Create point from found coordinates in image.
                                   vpImagePoint cog_image_coordinate = vpImagePoint (cog_x, cog_y);
              
                                   // Set intrinsic camera parameters, since we will have to convert image pixel 
                                   // coordinates to visual features expressed in meters.
                                   // vpCameraParameters(px,py,u0,v0) where:
                                   //   -px = horizontal pixel size. 
                                   //   -py = vertical pixel size.
                                   //   -u0 = X-Coordinate of image center.
                                   //   -v0 = Y-Coordinate of image center.
                                   vpCameraParameters camera_params(500, 500, 240, 320);

                                   // Create a vpFeaturePoint using a vpFeaturePoint and the parameters of the camera. 
                                   // The vpDot contains only the pixel coordinates of the point in an image. 
                                   // Thus this method uses the camera parameters to compute the meter coordinates 
                                   // in x and y in the image plane.
                                   vpFeatureBuilder::create(p_cog, camera_params, cog_image_coordinate);   

                                   // It is not possible to compute the depth of the point Z (meter) in the camera 
                                   // frame in the vpFeatureBuilder above.  
                                   // This coordinate is needed in vpFeaturePoint to compute the interaction matrix. 
                                   p_cog.set_Z(0.40);      

                                   // Compute the visual servoing velocity vector.
                                   vpColVector v = task.computeControlLaw();

                                   // Get feature error (s-s*) for the feature point. For this feature
                                   // point, we have 2 errors (along x and y axis).
                                   // This error is expressed in meters in the camera frame
                                   vpColVector error = task.getError(); 
                                   res.error_y = error[0];
                                   res.error_x = error[1];
                                   ROS_INFO("Feature error (x,y): (%0.5f , %0.5f)", res.error_x, res.error_y);  

                                   // Fill and send velocity vector as response to this service.
                                   std::vector<double> velocity_vector(6);
                                   velocity_vector[0] = v.operator[](0); 
                                   velocity_vector[1] = v.operator[](1); 
                                   velocity_vector[2] = v.operator[](2); 
                                   velocity_vector[3] = v.operator[](3); 
                                   velocity_vector[4] = v.operator[](4); 
                                   velocity_vector[5] = v.operator[](5); 
                                   
                                   // Send velocity vecotor as a response.
                                   res.servo_velocity_vector     = velocity_vector;
                                   res.found_vector              = true;
                                   res.initialized               = true; 
		                         ROS_INFO("Velocity vector has been sent: %f %f %f %f %f %f", velocity_vector[0], velocity_vector[1], velocity_vector[2], velocity_vector[3], velocity_vector[4], velocity_vector[5]);

                                   // End this servo iteration.  
                                   iteration++;
                                   ROS_INFO("Servo iteration performed, velocity vector send.");
                              }
                              else
                              {
                                   ROS_INFO("Servo iteration failed : could not find center of gravity. Retry.");                                                       
                              
                                   // Fill and send empty velocity vector as response to this service.
                                   std::vector<double> velocity_vector(6);
                                   res.servo_velocity_vector = velocity_vector;
                                   res.found_vector = false; 
                                   res.initialized = true;
                              }
                         }
                         else
                         {
                               if(!servo_initialized)
                               {
                                   ROS_INFO("Servo function not yet initialized.");
                                   res.found_vector = false;
                                   res.initialized = false; 
                               }
                               if(request_message != control_message )
                               {
                                   ROS_INFO("Wrong request message, use: %s", control_message.c_str() );
                                   res.found_vector = false;
                                   res.initialized = false; 
                               }
                         }
               }       
          }
          catch(vpException e) 
          {
               ROS_ERROR("Could not calculate velocity vector in getVelocityVector_Point");
               res.found_vector = false;
               res.initialized = false; 
          }     
     	program_lock = false;
		return true;
     }
	else
	{
		ROS_INFO("Visual servo calculation currently already executing and service (getVelocityVector_Point) is therefore locked.");
		return false;
	}

}
 



/***************************************************************************************************
 *
 *   Function requests camera parameters from repsonsible image acquisition node. 
 *   Flips a global boolean if loaded.
 *
 **************************************************************************************************/
void request_camera_parameters()
{
     std::string request_message = "trigger";
     image_acquisition::request_camera_parameters service;  
     service.request.request_message = request_message;  
                           
     if (camera_parameters_client.call(service))
     {                
          f  = service.response.f;
          k1 = service.response.k1;
          k2 = service.response.k2;  
          k3 = service.response.k3; 
          P1 = service.response.P1;  
          P2 = service.response.P2;  
          Sx = service.response.Sx;  
          Sy = service.response.Sy;  
          Cx = service.response.Cx;  
          Cy = service.response.Cy;
          Iw = service.response.Iw;
          Ih = service.response.Ih;

          camera_parameters_loaded = true;
     }
     else
     {
          ROS_INFO("Could not call camera parameter service. (Servo Node)");
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
 *	A service is created that can be called to return a 'velocity vector': the next step of 
 *	correction that the robot needs to perform to servo to the target. This vector is calculated
 *	by 1) taking an new image, 2) calculate the image features, 3) calculate the velocity vector
 * 	based on the image features. 
 *
 **************************************************************************************************/
int main(int argc, char **argv)
{
     // Initialize this rosnode with an empty name string. This allow the node's name
     // to be set in the launchfile so that multiple copies of this node with different
     // names can be launched. Although you would probably not run more than 1 servo node, 
     // it is good practice to initialize the ros node's name in the launch file. 
     ros::init(argc, argv, "");
     sleep(10);
     
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
	program_lock             = false;
	initialized              = false;
	servo_initialized        = false;
	camera_parameters_loaded = false;
	
     // Service for providing the main output from this node: a velocity vector.
     // Only one service can be activated at a time due to equal service names.
     velocity_vector_output_service = node_handle.advertiseService("request_servo_velocity_vector", getVelocityVector_Point); 

     // Initialize frame transformation parameters for simulated camera.
     tf::TransformBroadcaster      tf_broadcaster;
     tf::Transform                 transform;
     simulated_camera_position  =  tf::Vector3(0.0, 0.0, 0.0) ;
     simulated_camera_rotation  =  tf::Quaternion(0, 0, 0) ;

     // Get camera parameters from Application Control node required for servo algorithm.
     camera_parameters_client = node_handle.serviceClient<image_acquisition::request_camera_parameters>("/image_acquisition_node/request_camera_parameters");
     while(!camera_parameters_loaded)
     {
          request_camera_parameters();
          ros::Rate rate(1.0);
          rate.sleep();
     }

     // The ROS node will now enter the following loop.	
     ros::Rate rate(10.0);
     while (node_handle.ok())
     {
          // Frame transformations need to be updated constantly, otherwise they will fade out of existence. 
          transform.setOrigin(simulated_camera_position);
          transform.setRotation(simulated_camera_rotation);
          tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "simulated_camera_frame"));

          // Do one spin of the rosnode, e.g. checking and executing callbacks.
          // (http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
          ros::spinOnce();
          rate.sleep();
     }
	return 0;
}
