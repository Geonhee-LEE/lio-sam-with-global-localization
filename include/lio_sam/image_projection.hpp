#ifndef LIO_SAM_IMAGE_PROJECTION_HPP_
#define LIO_SAM_IMAGE_PROJECTION_HPP_

#include "utility.hpp"
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "lio_sam_with_global_localization/msg/cloud_info.hpp"
#include "lio_sam/utility.hpp"

namespace lio_sam {
struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection {
public:
  ImageProjection(rclcpp_lifecycle::LifecycleNode *node_,
                  std::shared_ptr<LioSamParams> params);
  virtual ~ImageProjection();

private:
  std::mutex imuLock;
  std::mutex odoLock;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
  rclcpp::Publisher<lio_sam_with_global_localization::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
  std::deque<sensor_msgs::msg::Imu> imuQueue;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
  rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
  std::deque<nav_msgs::msg::Odometry> odomQueue;

  std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
  sensor_msgs::msg::PointCloud2 currentCloudMsg;

  double *imuTime;
  double *imuRotX;
  double *imuRotY;
  double *imuRotZ;

  int imuPointerCur;
  bool firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr extractedCloud;

  int ringFlag;
  int deskewFlag;
  cv::Mat rangeMat;

  bool odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  lio_sam_with_global_localization::msg::CloudInfo cloudInfo;
  double timeScanCur;
  double timeScanEnd;
  std_msgs::msg::Header cloudHeader;

  std::vector<int> columnIdnCountVec;

  rclcpp_lifecycle::LifecycleNode *node_;
  std::shared_ptr<LioSamParams> params_;
  SensorType sensor_;

  void allocateMemory();
  void resetParameters();
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg);
  void cloudHandler(
      const sensor_msgs::msg::PointCloud2::SharedPtr PointTypelaserCloudMsg);
  bool cachePointCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg);
  bool deskewInfo();
  void imuDeskewInfo();
  void odomDeskewInfo();
  void findRotation(double pointTime, float *rotXCur, float *rotYCur,
                    float *rotZCur);
  void findPosition(double relTime, float *posXCur, float *posYCur,
                    float *posZCur);
  pcl::PointXYZI deskewPoint(pcl::PointXYZI *point, double relTime);
  void projectPointCloud();
  void cloudExtraction();
  void publishClouds();
};
} //  namespace lio_sam

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lio_sam::VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lio_sam::OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(
        uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

#endif //  LIO_SAM_IMAGE_PROJECTION_HPP_
