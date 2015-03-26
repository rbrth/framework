#ifndef LSD_SLAM_POINTCLOUD_PUBLISHER_H_
#define LSD_SLAM_POINTCLOUD_PUBLISHER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "slam_bridge/keyframeMsg.h"
#include <image_acquisition/request_camera_parameters.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Point32.h>
#include <sstream>
#include <fstream>
#include "robot_control/request_endpointstate.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "sophus/sim3.hpp"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class SlamBridge{

private:    

    struct InputPointDense
    {
        float idepth;
        float idepth_var;
        unsigned char color[4];
    };

    bool                        camera_parameters_loaded;
    int                         sparsity_factor;
    ros::ServiceClient          camera_parameters_client;
    ros::ServiceClient          robot_endpointstate_client;
    ros::Publisher              output_pointcloud_publisher;
    ros::Subscriber             raw_pointcloud_subcriber;
    ros::Subscriber             slam_pose_subcriber;
    InputPointDense*            originalInput;
    std::string                 frame;
    sensor_msgs::PointCloud2    reconstructed_scene_pointcloud2;
    bool                        lsd_slam_frame_transformed;

    Sophus::Sim3f               camToWorld;

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

public:
    ros::NodeHandle             node_handle;
    tf::TransformListener       transform_listener;
    geometry_msgs::PoseStamped  robot_endpointstate_pose;

    SlamBridge();
    void pointcloudCallback(const slam_bridge::keyframeMsg::ConstPtr& raw_point_cloud_message);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& slam_pose);
    bool requestEndPointState();
    bool requestCameraParameters();
};

#endif /* LSD_SLAM_POINTCLOUD_PUBLISHER_H_ */
