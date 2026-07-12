#ifndef LIO_SAM_MAP_OPTIMIZATION_HPP_
#define LIO_SAM_MAP_OPTIMIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "lio_sam_with_global_localization/msg/cloud_info.hpp"
#include "lio_sam_with_global_localization/srv/save_map.hpp"
#include "lio_sam/utility.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace lio_sam {
/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is
 * time stamp)
 */
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

typedef PointXYZIRPYT PointTypePose;

class MapOptimization {
public:
  MapOptimization(rclcpp_lifecycle::LifecycleNode *node_,
                  std::shared_ptr<LioSamParams> params);
  virtual ~MapOptimization();

  void loopClosureThread();
  void visualizeGlobalMapThread();
  void globalLocalizeThreadFunc();

private:
  // gtsam
  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;
  Eigen::MatrixXd poseCovariance;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudSurround;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pubLaserOdometryIncremental;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubHistoryKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubRecentKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubCloudRegisteredRaw;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pubLoopConstraintEdge;

  rclcpp::Service<lio_sam_with_global_localization::srv::SaveMap>::SharedPtr srvSaveMap;
  rclcpp::Subscription<lio_sam_with_global_localization::msg::CloudInfo>::SharedPtr subCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

  std::deque<nav_msgs::msg::Odometry> gpsQueue;
  lio_sam_with_global_localization::msg::CloudInfo cloudInfo;

  vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cornerCloudKeyFrames;
  vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> surfCloudKeyFrames;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

  pcl::PointCloud<pcl::PointXYZI>::Ptr
      laserCloudCornerLast; // corner feature set from odoOptimization
  pcl::PointCloud<pcl::PointXYZI>::Ptr
      laserCloudSurfLast; // surf feature set from odoOptimization
  pcl::PointCloud<pcl::PointXYZI>::Ptr
      laserCloudCornerLastDS; // downsampled corner feature set from
                              // odoOptimization
  pcl::PointCloud<pcl::PointXYZI>::Ptr
      laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;
  pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;

  std::vector<pcl::PointXYZI>
      laserCloudOriCornerVec; // corner point holder for parallel computation
  std::vector<pcl::PointXYZI> coeffSelCornerVec;
  std::vector<bool> laserCloudOriCornerFlag;
  std::vector<pcl::PointXYZI>
      laserCloudOriSurfVec; // surf point holder for parallel computation
  std::vector<pcl::PointXYZI> coeffSelSurfVec;
  std::vector<bool> laserCloudOriSurfFlag;

  map<int,
      pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZI>>>
      laserCloudMapContainer;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMapDS;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeHistoryKeyPoses;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterICP;
  pcl::VoxelGrid<pcl::PointXYZI>
      downSizeFilterSurroundingKeyPoses; // for surrounding key poses of
                                         // scan-to-map optimization

  rclcpp::Time timeLaserInfoStamp;
  double timeLaserInfoCur;

  float transformTobeMapped[6];

  std::mutex mtx;
  std::mutex mtxLoopInfo;

  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  int laserCloudCornerFromMapDSNum = 0;
  int laserCloudSurfFromMapDSNum = 0;
  int laserCloudCornerLastDSNum = 0;
  int laserCloudSurfLastDSNum = 0;

  bool aLoopIsClosed = false;
  map<int, int> loopIndexContainer; // from new to old
  vector<pair<int, int>> loopIndexQueue;
  vector<gtsam::Pose3> loopPoseQueue;
  vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
  deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

  nav_msgs::msg::Path globalPath;

  Eigen::Affine3f transPointAssociateToMap;
  Eigen::Affine3f incrementalOdometryAffineFront;
  Eigen::Affine3f incrementalOdometryAffineBack;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  // map->odom correction TF for downstream consumers (e.g. Nav2), updated by
  // global localization. Static (latched) because it changes at the slow
  // global-ICP rate and must stay valid between updates.
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> mapToOdomBroadcaster;
  void publishMapToOdomTf(float x, float y, float z, float roll, float pitch,
                          float yaw, const rclcpp::Time &stamp);

  rclcpp_lifecycle::LifecycleNode *node_;
  std::shared_ptr<LioSamParams> params_;
  SensorType sensor_;

  void allocateMemory();
  void laserCloudInfoHandler(const lio_sam_with_global_localization::msg::CloudInfo::SharedPtr msgIn);
  void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg);
  void pointAssociateToMap(pcl::PointXYZI const *const pi,
                           pcl::PointXYZI *const po);
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  transformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                      PointTypePose *transformIn);
  gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
  gtsam::Pose3 trans2gtsamPose(float transformIn[]);
  Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
  Eigen::Affine3f trans2Affine3f(float transformIn[]);
  PointTypePose trans2PointTypePose(float transformIn[]);

  void publishGlobalMap();

  void
  loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg);
  void performLoopClosure();
  bool detectLoopClosureDistance(int *latestID, int *closestID);
  bool detectLoopClosureExternal(int *latestID, int *closestID);
  void
  loopFindNearKeyframes(pcl::PointCloud<pcl::PointXYZI>::Ptr &nearKeyframes,
                        const int &key, const int &searchNum);
  void visualizeLoopClosure();
  void updateInitialGuess();
  void extractForLoopClosure();
  void extractNearby();
  void extractCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudToExtract);
  void extractSurroundingKeyFrames();
  void downsampleCurrentScan();
  void updatePointAssociateToMap();
  void cornerOptimization();
  void surfOptimization();
  void combineOptimizationCoeffs();
  bool LMOptimization(int iterCount);
  void scan2MapOptimization();
  void transformUpdate();
  float constraintTransformation(float value, float limit);
  bool saveFrame();
  void addOdomFactor();
  void addGPSFactor();
  void addLoopFactor();
  void saveKeyFramesAndFactor();
  void correctPoses();
  void updatePath(const PointTypePose &pose_in);
  void publishOdometry();
  void publishFrames();

  // Global Localization
  enum InitializedFlag {
    NonInitialized,
    Initializing,
    Initialized
  };
  InitializedFlag initializedFlag = NonInitialized;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudGlobalMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudGlobalMapDS;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudScanForInitialize;
  pcl::PointCloud<pcl::PointXYZI>::Ptr latestKeyFrameCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr nearHistoryKeyFrameCloud;

  std::mutex mtxWindow;
  std::vector<pcl::PointXYZI> window_cloudKeyPoses3D;
  std::vector<PointTypePose> window_cloudKeyPoses6D;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> window_cornerCloudKeyFrames;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> window_surfCloudKeyFrames;
  int winSize = 30;

  float transformInTheWorld[6] = {0};
  float transformOdomToWorld[6] = {0};
  std::mutex mtxtransformOdomToWorld;
  std::mutex mtx_general;

  int imuPreintegrationResetId = 0;
  int frameNum = 1;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapWorld;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudInWorld;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubOdomToMapPose;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subIniPoseFromRviz;

  std::unique_ptr<std::thread> globalLocalizeThreadPtr;

  void cloudGlobalLoad();
  void ICPLocalizeInitialize();
  void ICPscanMatchGlobal();
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);
  void extractForLocalization();
  void saveKeyFramesAndFactorLocalization();
  void publishFramesLocalization();
};
} // namespace lio_sam

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lio_sam::PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                 time))

#endif //  LIO_SAM_MAP_OPTIMIZATION_HPP_
