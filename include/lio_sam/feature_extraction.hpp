#ifndef LIO_SAM_FEATURE_EXTRACTION_HPP_
#define LIO_SAM_FEATURE_EXTRACTION_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "lio_sam_with_global_localization/msg/cloud_info.hpp"
#include "lio_sam/utility.hpp"

namespace lio_sam {
struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction {
public:
  FeatureExtraction(rclcpp_lifecycle::LifecycleNode *node_,
                    std::shared_ptr<LioSamParams> params);
  virtual ~FeatureExtraction();

  void laserCloudInfoHandler(const lio_sam_with_global_localization::msg::CloudInfo::SharedPtr msgIn);

private:
  void initializationValue();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
  void freeCloudInfoMemory();
  void publishFeatureCloud();

  rclcpp::Subscription<lio_sam_with_global_localization::msg::CloudInfo>::SharedPtr subLaserCloudInfo;
  rclcpp::Publisher<lio_sam_with_global_localization::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

  pcl::PointCloud<pcl::PointXYZI>::Ptr extractedCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cornerCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr surfaceCloud;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

  lio_sam_with_global_localization::msg::CloudInfo cloudInfo;
  std_msgs::msg::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  rclcpp_lifecycle::LifecycleNode *node_;
  std::shared_ptr<LioSamParams> params_;
};
} //  namespace lio_sam
#endif //  LIO_SAM_FEATURE_EXTRACTION_HPP_
