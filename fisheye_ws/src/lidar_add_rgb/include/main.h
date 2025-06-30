#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver/CustomMsg.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include<mutex>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rosbag/bag.h>
    int method;

    std::string frame_id;

    std::string topic_odom_in;
    std::string topic_lidar_in;
    std::string topic_lidar_out;
    std::string save_pcd;    

    ros::Subscriber sub_lidar;
    ros::Subscriber sub_camera_0;
    ros::Subscriber sub_camera_1;    
    ros::Subscriber sub_odom;    
    ros::Publisher pub_lidar;

    using PointType = pcl::PointXYZI;

    std::list<Eigen::Matrix4f> poses_list;
    std::vector<double> poses_time;
    std::vector<double> lidar_time;    
    std::vector<double> camera_time_0;
    std::vector<double> camera_time_1;    
    std::list<pcl::PointCloud<PointType>::Ptr> lidar_list;
    std::list<cv::Mat> camera_list_0;
    std::list<cv::Mat> camera_list_1;



  std::list<sensor_msgs::Image::ConstPtr> image0_msgs;
  std::list<nav_msgs::Odometry::ConstPtr> image0_odoms;
  std::list<sensor_msgs::Image::ConstPtr> image1_msgs;
  std::list<nav_msgs::Odometry::ConstPtr> image1_odoms;
  std::list<sensor_msgs::PointCloud2::ConstPtr> lidar_msgs;
  std::list<nav_msgs::Odometry::ConstPtr> lidar_odoms;

  std::map<size_t, Eigen::Matrix4d> I_L;
  std::map<size_t, Eigen::Matrix4d> I_C;  
  std::map<size_t, Eigen::Matrix4d> C_I;  

    std::vector<std::string> topic_camera_in;

    std::vector<std::vector<double>> distortion_coeffs;
    std::vector<std::vector<double>> intrinsics;
    std::vector<std::vector<int>> resolution;    
    std::vector<Eigen::Matrix4d> T_imu_cam;

    std::vector<Eigen::Matrix4f> TL2C;
    std::vector<Eigen::Matrix4f> TC2L;    
    std::vector<cv::Mat> camera_matrix; 
    std::vector<cv::Mat> D;

    int down_size;

    bool save_pcl;
    bool save_rosbag;
    bool save_assessment;
    std::string save_assessment_dir;    
    std::string save_rosbag_dir;    

    std::vector<pcl::PointCloud<PointType>::Ptr> save_pcds;
    std::vector<Eigen::Matrix4f> save_poses;    
    std::vector<double> save_time;        