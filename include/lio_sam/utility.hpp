#pragma once
#ifndef LIO_SAM__UTILITY_HPP_
#define LIO_SAM__UTILITY_HPP_

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/kdtree/kdtree_flann.h> // pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
                                     // is defined before
#define PCL_NO_PRECOMPILE
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std;
enum class SensorType { VELODYNE, OUSTER, LIVOX };
namespace lio_sam
{
struct LioSamParams {
  // Topic
  std::string point_cloud_topic;
  std::string imu_topic;
  std::string odom_topic;
  std::string gps_topic;

  // Frame
  std::string lidar_frame;
  std::string baselink_frame;
  std::string odometry_frame;
  std::string map_frame;

  // GPS Setting
  bool use_imu_heading_init;
  bool use_gps_elevation;
  float gps_covariance_threshold;
  float pose_covariance_threshold;

  // Export Setting
  bool save_pcd;
  std::string pointcloud_map_directory_path;

  // Sensor Setting
  std::string sensor;
  int vertical_scan_num;
  int horizontal_scan_num;
  int downsample_rate;
  float lidar_min_range;
  float lidar_max_range;

  // IMU Setting
  std::string imu_type;
  double imu_rate;
  double imu_acc_noise;
  double imu_gyro_noise;
  double imu_acc_bias_density;
  double imu_gyro_bias_density;
  double accel_lpf_cutoff_freq;
  double imu_gravity;
  float imu_rpy_weight;

  // IMU to LiDAR
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  // LOAM feature threshold
  float edge_threshold;
  float surf_threshold;
  int valid_edge_feature_num;
  int valid_surf_feature_num;

  // Voxel filter params
  float odometry_surf_leaf_size;
  float mapping_corner_leaf_size;
  float mapping_surf_leaf_size;

  // Robot motion constraint
  float z_tolerance;
  float rotation_tolerance;

  // CPU Params
  int number_of_cores;
  double mapping_process_interval;

  // Surrounding Map
  float surrounding_keyframe_distance_threshold;
  float surrounding_keyframe_angle_threshold;
  float surrounding_keyframe_density;
  float surrounding_keyframe_search_radius;

  // Loop closure
  bool loop_closure_enable_flag;
  float loop_closure_frequency;
  int surrounding_keyframe_size;
  float history_keyframe_search_radius;
  double history_keyframe_time_difference;
  int history_keyframe_search_num;
  double history_keyframe_fitness_score;
  bool scan_context_loop_closure_flag;
  double scan_context_limit_height;

  // Visualization
  bool global_map_visualization_flag;
  float global_map_visualization_search_radius;
  float global_map_visualization_pose_density;
  float global_map_visualization_leaf_size;

  // Global Localization
  bool global_localization_flag;
  std::string load_map_file_dir;

  // Debug log
  bool show_log;
};

extern rmw_qos_profile_t qos_profile;
extern rclcpp::QoS qos;
extern rmw_qos_profile_t qos_profile_imu;
extern rclcpp::QoS qos_imu;
extern rmw_qos_profile_t qos_profile_lidar;
extern rclcpp::QoS qos_lidar;

sensor_msgs::msg::Imu
imuConverter(const sensor_msgs::msg::Imu &imu_in,
             const Eigen::Matrix3d &lidar_to_imu_rotation,
             const Eigen::Quaterniond &imu_to_lidar_quaternion);

sensor_msgs::msg::PointCloud2 publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
    pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud, rclcpp::Time thisStamp,
    std::string thisFrame);

template <typename T> double stamp2Sec(const T &stamp) {
  return rclcpp::Time(stamp).seconds();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x,
                           T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y,
                       T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch,
                   T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(thisImuMsg->orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

float pointDistance(pcl::PointXYZI p);

float pointDistance(pcl::PointXYZI p1, pcl::PointXYZI p2);

void saveSCD(
  std::string fileName, Eigen::MatrixXd matrix,
  std::string delimiter = " ");

std::string padZeros(int val, int num_digits = 6);
} //  namespace lio_sam
#endif // LIO_SAM__UTILITY_HPP_
