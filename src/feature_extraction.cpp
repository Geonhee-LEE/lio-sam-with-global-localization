#include "lio_sam/feature_extraction.hpp"

namespace lio_sam {
FeatureExtraction::FeatureExtraction(rclcpp_lifecycle::LifecycleNode *node,
                                     std::shared_ptr<LioSamParams> params)
    : node_(node), params_(params) {
  subLaserCloudInfo = node_->create_subscription<lio_sam_with_global_localization::msg::CloudInfo>(
      "lio_sam/deskew/cloud_info", qos,
      std::bind(&FeatureExtraction::laserCloudInfoHandler, this,
                std::placeholders::_1));

  pubLaserCloudInfo = node_->create_publisher<lio_sam_with_global_localization::msg::CloudInfo>(
      "lio_sam/feature/cloud_info", qos);
  pubCornerPoints = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/feature/cloud_corner", 1);
  pubSurfacePoints = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/feature/cloud_surface", 1);

  initializationValue();
}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::initializationValue() {
  cloudSmoothness.resize(params_->vertical_scan_num *
                         params_->horizontal_scan_num);

  downSizeFilter.setLeafSize(params_->odometry_surf_leaf_size,
                             params_->odometry_surf_leaf_size,
                             params_->odometry_surf_leaf_size);

  extractedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cornerCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  surfaceCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  cloudCurvature =
      new float[params_->vertical_scan_num * params_->horizontal_scan_num];
  cloudNeighborPicked =
      new int[params_->vertical_scan_num * params_->horizontal_scan_num];
  cloudLabel =
      new int[params_->vertical_scan_num * params_->horizontal_scan_num];
}

void FeatureExtraction::laserCloudInfoHandler(
    const lio_sam_with_global_localization::msg::CloudInfo::SharedPtr msgIn) {
  cloudInfo = *msgIn;
  cloudHeader = msgIn->header;
  pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);

  calculateSmoothness();
  markOccludedPoints();
  extractFeatures();
  publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness() {
  int cloudSize = extractedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange =
        cloudInfo.point_range[i - 5] + cloudInfo.point_range[i - 4] +
        cloudInfo.point_range[i - 3] + cloudInfo.point_range[i - 2] +
        cloudInfo.point_range[i - 1] - cloudInfo.point_range[i] * 10 +
        cloudInfo.point_range[i + 1] + cloudInfo.point_range[i + 2] +
        cloudInfo.point_range[i + 3] + cloudInfo.point_range[i + 4] +
        cloudInfo.point_range[i + 5];

    cloudCurvature[i] = diffRange * diffRange;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureExtraction::markOccludedPoints() {
  int cloudSize = extractedCloud->points.size();
  for (int i = 5; i < cloudSize - 6; ++i) {
    float depth1 = cloudInfo.point_range[i];
    float depth2 = cloudInfo.point_range[i + 1];
    int columnDiff = std::abs(
        int(cloudInfo.point_col_ind[i + 1] - cloudInfo.point_col_ind[i]));
    if (columnDiff < 10) {
      if (depth1 - depth2 > 0.3) {
        for (int l = -5; l <= 0; ++l)
          cloudNeighborPicked[i + l] = 1;
      } else if (depth2 - depth1 > 0.3) {
        for (int l = 1; l <= 6; ++l)
          cloudNeighborPicked[i + l] = 1;
      }
    }
    float diff1 = std::abs(
        float(cloudInfo.point_range[i - 1] - cloudInfo.point_range[i]));
    float diff2 = std::abs(
        float(cloudInfo.point_range[i + 1] - cloudInfo.point_range[i]));

    if (diff1 > 0.02 * cloudInfo.point_range[i] &&
        diff2 > 0.02 * cloudInfo.point_range[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureExtraction::extractFeatures() {
  cornerCloud->clear();
  surfaceCloud->clear();

  pcl::PointCloud<pcl::PointXYZI>::Ptr surfaceCloudScan(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surfaceCloudScanDS(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (int i = 0; i < params_->vertical_scan_num; i++) {
    surfaceCloudScan->clear();

    for (int j = 0; j < 6; j++) {
      int sp = (cloudInfo.start_ring_index[i] * (6 - j) +
                cloudInfo.end_ring_index[i] * j) /
               6;
      int ep = (cloudInfo.start_ring_index[i] * (5 - j) +
                cloudInfo.end_ring_index[i] * (j + 1)) /
                   6 -
               1;

      if (sp >= ep)
        continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > params_->edge_threshold) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerCloud->push_back(extractedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff =
                std::abs(int(cloudInfo.point_col_ind[ind + l] -
                             cloudInfo.point_col_ind[ind + l - 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff =
                std::abs(int(cloudInfo.point_col_ind[ind + l] -
                             cloudInfo.point_col_ind[ind + l + 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < params_->surf_threshold) {
          cloudLabel[ind] = -1;
          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            int columnDiff =
                std::abs(int(cloudInfo.point_col_ind[ind + l] -
                             cloudInfo.point_col_ind[ind + l - 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff =
                std::abs(int(cloudInfo.point_col_ind[ind + l] -
                             cloudInfo.point_col_ind[ind + l + 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfaceCloudScan->push_back(extractedCloud->points[k]);
        }
      }
    }

    surfaceCloudScanDS->clear();
    downSizeFilter.setInputCloud(surfaceCloudScan);
    downSizeFilter.filter(*surfaceCloudScanDS);

    *surfaceCloud += *surfaceCloudScanDS;
  }
}

void FeatureExtraction::freeCloudInfoMemory() {
  cloudInfo.start_ring_index.clear();
  cloudInfo.end_ring_index.clear();
  cloudInfo.point_col_ind.clear();
  cloudInfo.point_range.clear();
}

void FeatureExtraction::publishFeatureCloud() {
  freeCloudInfoMemory();
  cloudInfo.cloud_corner = publishCloud(
      pubCornerPoints, cornerCloud, cloudHeader.stamp, params_->lidar_frame);
  cloudInfo.cloud_surface = publishCloud(
      pubSurfacePoints, surfaceCloud, cloudHeader.stamp, params_->lidar_frame);
  pubLaserCloudInfo->publish(cloudInfo);
}
} // namespace lio_sam

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions options;
//   options.use_intra_process_comms(true);
//   rclcpp::executors::SingleThreadedExecutor exec;

//   auto FE = std::make_shared<lio_sam::FeatureExtraction>(options);

//   exec.add_node(FE);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Feature
//   Extraction Started.\033[0m");

//   exec.spin();

//   rclcpp::shutdown();
//   return 0;
// }