#include "lio_sam/map_optimization.hpp"

namespace lio_sam {
MapOptimization::MapOptimization(rclcpp_lifecycle::LifecycleNode *node_,
                                 std::shared_ptr<LioSamParams> params)
    : node_(node_), params_(params) {
  if (params_->sensor == "velodyne") {
    sensor_ = SensorType::VELODYNE;
  } else if (params_->sensor == "ouster") {
    sensor_ = SensorType::OUSTER;
  } else if (params_->sensor == "livox") {
    sensor_ = SensorType::LIVOX;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Unknown sensor type! Set to Velodyne as default.");
    rclcpp::shutdown();
  }

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);

  pubKeyPoses = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/mapping/trajectory", 1);
  pubLaserCloudSurround =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "lio_sam/mapping/map_global", 1);
  pubLaserOdometryGlobal = node_->create_publisher<nav_msgs::msg::Odometry>(
      "lio_sam/mapping/odometry", qos);
  pubLaserOdometryIncremental =
      node_->create_publisher<nav_msgs::msg::Odometry>(
          "lio_sam/mapping/odometry_incremental", qos);
  pubPath =
      node_->create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
  br = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  subCloud = node_->create_subscription<lio_sam_with_global_localization::msg::CloudInfo>(
      "lio_sam/feature/cloud_info", qos,
      std::bind(&MapOptimization::laserCloudInfoHandler, this,
                std::placeholders::_1));
  subGPS = node_->create_subscription<nav_msgs::msg::Odometry>(
      params_->gps_topic, 200,
      std::bind(&MapOptimization::gpsHandler, this, std::placeholders::_1));
  subLoop = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "lio_loop/loop_closure_detection", qos,
      std::bind(&MapOptimization::loopInfoHandler, this,
                std::placeholders::_1));

  auto saveMapService =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<lio_sam_with_global_localization::srv::SaveMap::Request> req,
             std::shared_ptr<lio_sam_with_global_localization::srv::SaveMap::Response> res) -> void {
    (void)request_header;
    string saveMapDirectory;
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if (req->destination.empty())
      saveMapDirectory =
          std::getenv("HOME") + params_->pointcloud_map_directory_path;
    else
      saveMapDirectory = std::getenv("HOME") + req->destination;
    cout << "Save destination: " << saveMapDirectory << endl;
    if (cloudKeyPoses3D->empty()) {
      cout << "save_map: no keyframes, aborting save (existing map kept)."
           << endl;
      res->success = false;
      return;
    }
    // Write into a temp directory first and swap at the end, so an
    // interrupted save cannot destroy the previous map.
    if (!saveMapDirectory.empty() && saveMapDirectory.back() == '/')
      saveMapDirectory.pop_back();
    const string finalMapDirectory = saveMapDirectory;
    saveMapDirectory += ".saving";
    (void)system((std::string("exec rm -rf ") + saveMapDirectory).c_str());
    (void)system((std::string("mkdir -p ") + saveMapDirectory).c_str());
    // save key frame transformations
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd",
                               *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd",
                               *cloudKeyPoses6D);
    // extract global point cloud map
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalCornerCloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalCornerCloudDS(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalSurfCloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalSurfCloudDS(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapCloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
      *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],
                                                 &cloudKeyPoses6D->points[i]);
      *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i],
                                               &cloudKeyPoses6D->points[i]);
      cout << "\r" << std::flush << "Processing feature cloud " << i << " of "
           << cloudKeyPoses6D->size() << " ...";
    }
    if (req->resolution != 0) {
      cout << "\n\nSave resolution: " << req->resolution << endl;
      // down-sample and save corner cloud
      downSizeFilterCorner.setInputCloud(globalCornerCloud);
      downSizeFilterCorner.setLeafSize(req->resolution, req->resolution,
                                       req->resolution);
      downSizeFilterCorner.filter(*globalCornerCloudDS);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd",
                                 *globalCornerCloudDS);
      // down-sample and save surf cloud
      downSizeFilterSurf.setInputCloud(globalSurfCloud);
      downSizeFilterSurf.setLeafSize(req->resolution, req->resolution,
                                     req->resolution);
      downSizeFilterSurf.filter(*globalSurfCloudDS);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd",
                                 *globalSurfCloudDS);
    } else {
      // save corner cloud
      pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd",
                                 *globalCornerCloud);
      // save surf cloud
      pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd",
                                 *globalSurfCloud);
    }
    // save global point cloud map
    *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;
    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd",
                                         *globalMapCloud);
    if (ret == 0) {
      (void)system((std::string("exec rm -rf ") + finalMapDirectory).c_str());
      (void)system((std::string("mv ") + saveMapDirectory + " " +
                    finalMapDirectory).c_str());
    }
    res->success = ret == 0;
    downSizeFilterCorner.setLeafSize(params_->mapping_corner_leaf_size,
                                     params_->mapping_corner_leaf_size,
                                     params_->mapping_corner_leaf_size);
    downSizeFilterSurf.setLeafSize(params_->mapping_surf_leaf_size,
                                   params_->mapping_surf_leaf_size,
                                   params_->mapping_surf_leaf_size);
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;
    return;
  };

  srvSaveMap = node_->create_service<lio_sam_with_global_localization::srv::SaveMap>("lio_sam/save_map",
                                                            saveMapService);
  pubHistoryKeyFrames = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/mapping/icp_loop_closure_history_cloud", 1);
  pubIcpKeyFrames = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/mapping/icp_loop_closure_history_cloud", 1);
  pubLoopConstraintEdge =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/lio_sam/mapping/loop_closure_constraints", 1);

  pubRecentKeyFrames = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/mapping/map_local", 1);
  pubRecentKeyFrame = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/mapping/cloud_registered", 1);
  pubCloudRegisteredRaw =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "lio_sam/mapping/cloud_registered_raw", 1);

  downSizeFilterCorner.setLeafSize(params_->mapping_corner_leaf_size,
                                   params_->mapping_corner_leaf_size,
                                   params_->mapping_corner_leaf_size);
  downSizeFilterSurf.setLeafSize(params_->mapping_surf_leaf_size,
                                 params_->mapping_surf_leaf_size,
                                 params_->mapping_surf_leaf_size);
  downSizeFilterICP.setLeafSize(params_->mapping_surf_leaf_size,
                                params_->mapping_surf_leaf_size,
                                params_->mapping_surf_leaf_size);
  downSizeFilterSurroundingKeyPoses.setLeafSize(
      params_->surrounding_keyframe_density,
      params_->surrounding_keyframe_density,
      params_->surrounding_keyframe_density); // for surrounding key poses of
                                              // scan-to-map optimization

  allocateMemory();

  if (params_->global_localization_flag) {
    subIniPoseFromRviz =
        node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", qos,
            std::bind(&MapOptimization::initialPoseCallback, this,
                      std::placeholders::_1));
    pubMapWorld = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/mapping/cloud_map_map", 1);
    pubLaserCloudInWorld =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/mapping/lasercloud_in_world", 1);
    pubOdomToMapPose =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "lio_sam/mapping/pose_odomTo_map", 1);
    pubGlobalMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/localization/global_map", 1);

    mapToOdomBroadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    // Nav2 needs a complete map->odom->base_link chain before the first
    // relocalization succeeds; start from identity, updated on every match.
    publishMapToOdomTf(0, 0, 0, 0, 0, 0, node_->get_clock()->now());

    cloudGlobalMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudGlobalMapDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudScanForInitialize.reset(new pcl::PointCloud<pcl::PointXYZI>());
    latestKeyFrameCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    nearHistoryKeyFrameCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    cloudGlobalLoad();
  }
}

MapOptimization::~MapOptimization() {}

void MapOptimization::allocateMemory() {
  cloudKeyPoses3D.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  copy_cloudKeyPoses3D.reset(new pcl::PointCloud<pcl::PointXYZI>());
  copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

  kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  laserCloudCornerLast.reset(
      new pcl::PointCloud<pcl::PointXYZI>()); // corner feature set from
                                              // odoOptimization
  laserCloudSurfLast.reset(
      new pcl::PointCloud<pcl::PointXYZI>()); // surf feature set from
                                              // odoOptimization
  laserCloudCornerLastDS.reset(
      new pcl::PointCloud<pcl::PointXYZI>()); // downsampled corner featuer set
                                              // from odoOptimization
  laserCloudSurfLastDS.reset(
      new pcl::PointCloud<pcl::PointXYZI>()); // downsampled surf featuer set
                                              // from odoOptimization

  laserCloudOri.reset(new pcl::PointCloud<pcl::PointXYZI>());
  coeffSel.reset(new pcl::PointCloud<pcl::PointXYZI>());

  laserCloudOriCornerVec.resize(params_->vertical_scan_num *
                                params_->horizontal_scan_num);
  coeffSelCornerVec.resize(params_->vertical_scan_num *
                           params_->horizontal_scan_num);
  laserCloudOriCornerFlag.resize(params_->vertical_scan_num *
                                 params_->horizontal_scan_num);
  laserCloudOriSurfVec.resize(params_->vertical_scan_num *
                              params_->horizontal_scan_num);
  coeffSelSurfVec.resize(params_->vertical_scan_num *
                         params_->horizontal_scan_num);
  laserCloudOriSurfFlag.resize(params_->vertical_scan_num *
                               params_->horizontal_scan_num);

  std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
            false);
  std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

  laserCloudCornerFromMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudCornerFromMapDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudSurfFromMapDS.reset(new pcl::PointCloud<pcl::PointXYZI>());

  kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  for (int i = 0; i < 6; ++i) {
    transformTobeMapped[i] = 0;
  }

  matP.setZero();
}

void MapOptimization::laserCloudInfoHandler(
    const lio_sam_with_global_localization::msg::CloudInfo::SharedPtr msgIn) {
  timeLaserInfoStamp = msgIn->header.stamp;
  timeLaserInfoCur = stamp2Sec(msgIn->header.stamp);

  cloudInfo = *msgIn;
  pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
  pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

  if (params_->global_localization_flag) {
    if (initializedFlag == NonInitialized || initializedFlag == Initializing) {
      if (cloudScanForInitialize->points.size() == 0) {
        downsampleCurrentScan();
        mtx_general.lock();
        *cloudScanForInitialize += *laserCloudCornerLastDS;
        *cloudScanForInitialize += *laserCloudSurfLastDS;
        mtx_general.unlock();

        laserCloudCornerLastDS->clear();
        laserCloudSurfLastDS->clear();
        laserCloudCornerLastDSNum = 0;
        laserCloudSurfLastDSNum = 0;

        transformTobeMapped[0] = cloudInfo.imu_roll_init;
        transformTobeMapped[1] = cloudInfo.imu_pitch_init;
        transformTobeMapped[2] = cloudInfo.imu_yaw_init;
        if (!params_->use_imu_heading_init) {
          transformTobeMapped[2] = 0;
        }
      }
      return;
    }

    frameNum++;

    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >=
        params_->mapping_process_interval) {
      timeLastProcessing = timeLaserInfoCur;

      updateInitialGuess();

      extractForLocalization();

      downsampleCurrentScan();

      scan2MapOptimization();

      saveKeyFramesAndFactorLocalization();

      publishOdometry();

      publishFramesLocalization();
    }
  } else {
    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >=
        params_->mapping_process_interval) {
      timeLastProcessing = timeLaserInfoCur;

      updateInitialGuess();

      extractSurroundingKeyFrames();

      downsampleCurrentScan();

      scan2MapOptimization();

      saveKeyFramesAndFactor();

      correctPoses();

      publishOdometry();

      publishFrames();
    }
  }
}

void MapOptimization::gpsHandler(
    const nav_msgs::msg::Odometry::SharedPtr gpsMsg) {
  gpsQueue.push_back(*gpsMsg);
}

void MapOptimization::pointAssociateToMap(pcl::PointXYZI const *const pi,
                                          pcl::PointXYZI *const po) {
  po->x = transPointAssociateToMap(0, 0) * pi->x +
          transPointAssociateToMap(0, 1) * pi->y +
          transPointAssociateToMap(0, 2) * pi->z +
          transPointAssociateToMap(0, 3);
  po->y = transPointAssociateToMap(1, 0) * pi->x +
          transPointAssociateToMap(1, 1) * pi->y +
          transPointAssociateToMap(1, 2) * pi->z +
          transPointAssociateToMap(1, 3);
  po->z = transPointAssociateToMap(2, 0) * pi->x +
          transPointAssociateToMap(2, 1) * pi->y +
          transPointAssociateToMap(2, 2) * pi->z +
          transPointAssociateToMap(2, 3);
  po->intensity = pi->intensity;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, PointTypePose *transformIn) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
      new pcl::PointCloud<pcl::PointXYZI>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(
      transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
      transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(params_->number_of_cores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                            transCur(0, 1) * pointFrom.y +
                            transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                            transCur(1, 1) * pointFrom.y +
                            transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                            transCur(2, 1) * pointFrom.y +
                            transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
}

gtsam::Pose3 MapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll),
                                          double(thisPoint.pitch),
                                          double(thisPoint.yaw)),
                      gtsam::Point3(double(thisPoint.x), double(thisPoint.y),
                                    double(thisPoint.z)));
}

gtsam::Pose3 MapOptimization::trans2gtsamPose(float transformIn[]) {
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
      gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

Eigen::Affine3f MapOptimization::pclPointToAffine3f(PointTypePose thisPoint) {
  return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f MapOptimization::trans2Affine3f(float transformIn[]) {
  return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5],
                                transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose MapOptimization::trans2PointTypePose(float transformIn[]) {
  PointTypePose thisPose6D;
  thisPose6D.x = transformIn[3];
  thisPose6D.y = transformIn[4];
  thisPose6D.z = transformIn[5];
  thisPose6D.roll = transformIn[0];
  thisPose6D.pitch = transformIn[1];
  thisPose6D.yaw = transformIn[2];
  return thisPose6D;
}

void MapOptimization::visualizeGlobalMapThread() {
  rclcpp::Rate rate(0.2);
  while (rclcpp::ok()) {
    rate.sleep();
    publishGlobalMap();
  }
  if (params_->save_pcd == false)
    return;
  if (cloudKeyPoses3D->empty()) {
    cout << "No keyframes to save, keeping existing map files." << endl;
    return;
  }
  cout << "****************************************************" << endl;
  cout << "Saving map to pcd files ..." << endl;
  params_->pointcloud_map_directory_path =
      std::getenv("HOME") + params_->pointcloud_map_directory_path;
  // Write into a temp directory first and swap at the end, so an interrupted
  // save cannot destroy the previous map.
  std::string finalMapDirectory = params_->pointcloud_map_directory_path;
  if (!finalMapDirectory.empty() && finalMapDirectory.back() == '/')
    finalMapDirectory.pop_back();
  params_->pointcloud_map_directory_path = finalMapDirectory + ".saving/";
  (void)system(
      (std::string("exec rm -rf ") + params_->pointcloud_map_directory_path)
          .c_str());
  (void)system(
      (std::string("mkdir -p ") + params_->pointcloud_map_directory_path).c_str());
  pcl::io::savePCDFileASCII(params_->pointcloud_map_directory_path +
                                "trajectory.pcd",
                            *cloudKeyPoses3D);
  pcl::io::savePCDFileASCII(params_->pointcloud_map_directory_path +
                                "transformations.pcd",
                            *cloudKeyPoses6D);
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalCornerCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalCornerCloudDS(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalSurfCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalSurfCloudDS(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
    *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],
                                               &cloudKeyPoses6D->points[i]);
    *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i],
                                             &cloudKeyPoses6D->points[i]);
    cout << "\r" << std::flush << "Processing feature cloud " << i << " of "
         << cloudKeyPoses6D->size() << " ...";
  }
  downSizeFilterCorner.setInputCloud(globalCornerCloud);
  downSizeFilterCorner.filter(*globalCornerCloudDS);
  pcl::io::savePCDFileASCII(params_->pointcloud_map_directory_path +
                                "cloudCorner.pcd",
                            *globalCornerCloudDS);
  downSizeFilterSurf.setInputCloud(globalSurfCloud);
  downSizeFilterSurf.filter(*globalSurfCloudDS);
  pcl::io::savePCDFileASCII(params_->pointcloud_map_directory_path +
                                "cloudSurf.pcd",
                            *globalSurfCloudDS);
  *globalMapCloud += *globalCornerCloud;
  *globalMapCloud += *globalSurfCloud;
  pcl::io::savePCDFileASCII(params_->pointcloud_map_directory_path +
                                "cloudGlobal.pcd",
                            *globalMapCloud);
  {
    std::string tmpDirectory = params_->pointcloud_map_directory_path;
    if (!tmpDirectory.empty() && tmpDirectory.back() == '/')
      tmpDirectory.pop_back();
    (void)system((std::string("exec rm -rf ") + finalMapDirectory).c_str());
    (void)system(
        (std::string("mv ") + tmpDirectory + " " + finalMapDirectory).c_str());
  }
  cout << "****************************************************" << endl;
  cout << "Saving map to pcd files completed" << endl;
}

void MapOptimization::publishGlobalMap() {
  if (pubLaserCloudSurround->get_subscription_count() == 0)
    return;

  if (cloudKeyPoses3D->points.empty() == true)
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeGlobalMap(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  ;
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapKeyPoses(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapKeyPosesDS(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapKeyFrames(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapKeyFramesDS(
      new pcl::PointCloud<pcl::PointXYZI>());

  // kd-tree to find near key frames to visualize
  std::vector<int> pointSearchIndGlobalMap;
  std::vector<float> pointSearchSqDisGlobalMap;
  // search near key frames to visualize
  mtx.lock();
  kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
  kdtreeGlobalMap->radiusSearch(
      cloudKeyPoses3D->back(), params_->global_map_visualization_search_radius,
      pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
  mtx.unlock();

  for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
    globalMapKeyPoses->push_back(
        cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
  // downsample near selected key frames
  pcl::VoxelGrid<pcl::PointXYZI>
      downSizeFilterGlobalMapKeyPoses; // for global map visualization
  downSizeFilterGlobalMapKeyPoses.setLeafSize(
      params_->global_map_visualization_pose_density,
      params_->global_map_visualization_pose_density,
      params_->global_map_visualization_pose_density); // for global map
                                                       // visualization
  downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
  downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
  for (auto &pt : globalMapKeyPosesDS->points) {
    kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap,
                                    pointSearchSqDisGlobalMap);
    pt.intensity =
        cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
  }

  // extract visualized and downsampled key frames
  for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
    if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) >
        params_->global_map_visualization_search_radius)
      continue;
    int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
    *globalMapKeyFrames += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames += *transformPointCloud(
        surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
  }
  // downsample visualized points
  pcl::VoxelGrid<pcl::PointXYZI>
      downSizeFilterGlobalMapKeyFrames; // for global map visualization
  downSizeFilterGlobalMapKeyFrames.setLeafSize(
      params_->global_map_visualization_leaf_size,
      params_->global_map_visualization_leaf_size,
      params_
          ->global_map_visualization_leaf_size); // for global map visualization
  downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
  downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
  publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp,
               params_->odometry_frame);
}

void MapOptimization::loopClosureThread() {
  if (params_->loop_closure_enable_flag == false)
    return;

  rclcpp::Rate rate(params_->loop_closure_frequency);
  while (rclcpp::ok()) {
    rate.sleep();
    performLoopClosure();
    visualizeLoopClosure();
  }
}

void MapOptimization::loopInfoHandler(
    const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg) {
  std::lock_guard<std::mutex> lock(mtxLoopInfo);
  if (loopMsg->data.size() != 2)
    return;

  loopInfoVec.push_back(*loopMsg);

  while (loopInfoVec.size() > 5)
    loopInfoVec.pop_front();
}

void MapOptimization::performLoopClosure() {
  if (cloudKeyPoses3D->points.empty() == true)
    return;

  mtx.lock();
  *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
  *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
  mtx.unlock();

  // find keys
  int loopKeyCur;
  int loopKeyPre;
  if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
      return;

  // extract cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr prevKeyframeCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  {
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                          params_->history_keyframe_search_num);
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
      return;
    if (pubHistoryKeyFrames->get_subscription_count() != 0)
      publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
                   params_->odometry_frame);
  }

  // ICP Settings
  static pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(params_->history_keyframe_search_radius * 2);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // Align clouds
  icp.setInputSource(cureKeyframeCloud);
  icp.setInputTarget(prevKeyframeCloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(
      new pcl::PointCloud<pcl::PointXYZI>());
  icp.align(*unused_result);

  if (icp.hasConverged() == false ||
      icp.getFitnessScore() > params_->history_keyframe_fitness_score)
    return;

  // publish corrected cloud
  if (pubIcpKeyFrames->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr closed_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
                             icp.getFinalTransformation());
    publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp,
                 params_->odometry_frame);
  }

  // Get pose transformation
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionLidarFrame;
  correctionLidarFrame = icp.getFinalTransformation();
  // transform from world origin to wrong pose
  Eigen::Affine3f tWrong =
      pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
  // transform from world origin to corrected pose
  Eigen::Affine3f tCorrect =
      correctionLidarFrame *
      tWrong; // pre-multiplying -> successive rotation about a fixed frame
  pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 poseFrom =
      Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo =
      pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
  gtsam::Vector Vector6(6);
  float noiseScore = icp.getFitnessScore();
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
      noiseScore;
  noiseModel::Diagonal::shared_ptr constraintNoise =
      noiseModel::Diagonal::Variances(Vector6);

  // Add pose constraint
  mtx.lock();
  loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
  loopPoseQueue.push_back(poseFrom.between(poseTo));
  loopNoiseQueue.push_back(constraintNoise);
  mtx.unlock();

  // add loop constriant
  loopIndexContainer[loopKeyCur] = loopKeyPre;
}

bool MapOptimization::detectLoopClosureDistance(int *latestID, int *closestID) {
  int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
  int loopKeyPre = -1;

  // check loop constraint added before
  auto it = loopIndexContainer.find(loopKeyCur);
  if (it != loopIndexContainer.end())
    return false;

  // find the closest history key frame
  std::vector<int> pointSearchIndLoop;
  std::vector<float> pointSearchSqDisLoop;
  kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
  kdtreeHistoryKeyPoses->radiusSearch(
      copy_cloudKeyPoses3D->back(), params_->history_keyframe_search_radius,
      pointSearchIndLoop, pointSearchSqDisLoop, 0);

  for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
    int id = pointSearchIndLoop[i];
    if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) >
        params_->history_keyframe_time_difference) {
      loopKeyPre = id;
      break;
    }
  }

  if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
    return false;

  *latestID = loopKeyCur;
  *closestID = loopKeyPre;

  return true;
}

bool MapOptimization::detectLoopClosureExternal(int *latestID, int *closestID) {
  // this function is not used yet, please ignore it
  int loopKeyCur = -1;
  int loopKeyPre = -1;

  std::lock_guard<std::mutex> lock(mtxLoopInfo);
  if (loopInfoVec.empty())
    return false;

  double loopTimeCur = loopInfoVec.front().data[0];
  double loopTimePre = loopInfoVec.front().data[1];
  loopInfoVec.pop_front();

  if (abs(loopTimeCur - loopTimePre) <
      params_->history_keyframe_time_difference)
    return false;

  int cloudSize = copy_cloudKeyPoses6D->size();
  if (cloudSize < 2)
    return false;

  // latest key
  loopKeyCur = cloudSize - 1;
  for (int i = cloudSize - 1; i >= 0; --i) {
    if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
      loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
    else
      break;
  }

  // previous key
  loopKeyPre = 0;
  for (int i = 0; i < cloudSize; ++i) {
    if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
      loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
    else
      break;
  }

  if (loopKeyCur == loopKeyPre)
    return false;

  auto it = loopIndexContainer.find(loopKeyCur);
  if (it != loopIndexContainer.end())
    return false;

  *latestID = loopKeyCur;
  *closestID = loopKeyPre;

  return true;
}

void MapOptimization::loopFindNearKeyframes(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &nearKeyframes, const int &key,
    const int &searchNum) {
  // extract near keyframes
  nearKeyframes->clear();
  int cloudSize = copy_cloudKeyPoses6D->size();
  for (int i = -searchNum; i <= searchNum; ++i) {
    int keyNear = key + i;
    if (keyNear < 0 || keyNear >= cloudSize)
      continue;
    *nearKeyframes += *transformPointCloud(
        cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
    *nearKeyframes += *transformPointCloud(
        surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
  }

  if (nearKeyframes->empty())
    return;

  // downsample near keyframes
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(
      new pcl::PointCloud<pcl::PointXYZI>());
  downSizeFilterICP.setInputCloud(nearKeyframes);
  downSizeFilterICP.filter(*cloud_temp);
  *nearKeyframes = *cloud_temp;
}

void MapOptimization::visualizeLoopClosure() {
  if (loopIndexContainer.empty())
    return;

  visualization_msgs::msg::MarkerArray markerArray;
  // loop nodes
  visualization_msgs::msg::Marker markerNode;
  markerNode.header.frame_id = params_->odometry_frame;
  markerNode.header.stamp = timeLaserInfoStamp;
  markerNode.action = visualization_msgs::msg::Marker::ADD;
  markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  markerNode.ns = "loop_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.3;
  markerNode.scale.y = 0.3;
  markerNode.scale.z = 0.3;
  markerNode.color.r = 0;
  markerNode.color.g = 0.8;
  markerNode.color.b = 1;
  markerNode.color.a = 1;
  // loop edges
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = params_->odometry_frame;
  markerEdge.header.stamp = timeLaserInfoStamp;
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.ns = "loop_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.1;
  markerEdge.color.r = 0.9;
  markerEdge.color.g = 0.9;
  markerEdge.color.b = 0;
  markerEdge.color.a = 1;

  for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end();
       ++it) {
    int key_cur = it->first;
    int key_pre = it->second;
    geometry_msgs::msg::Point p;
    p.x = copy_cloudKeyPoses6D->points[key_cur].x;
    p.y = copy_cloudKeyPoses6D->points[key_cur].y;
    p.z = copy_cloudKeyPoses6D->points[key_cur].z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
    p.x = copy_cloudKeyPoses6D->points[key_pre].x;
    p.y = copy_cloudKeyPoses6D->points[key_pre].y;
    p.z = copy_cloudKeyPoses6D->points[key_pre].z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
  }

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  pubLoopConstraintEdge->publish(markerArray);
}

void MapOptimization::updateInitialGuess() {
  // save current transformation before any processing
  incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

  static Eigen::Affine3f lastImuTransformation;
  // initialization
  if (cloudKeyPoses3D->points.empty()) {
    transformTobeMapped[0] = cloudInfo.imu_roll_init;
    transformTobeMapped[1] = cloudInfo.imu_pitch_init;
    transformTobeMapped[2] = cloudInfo.imu_yaw_init;

    if (!params_->use_imu_heading_init)
      transformTobeMapped[2] = 0;

    lastImuTransformation = pcl::getTransformation(
        0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init,
        cloudInfo.imu_yaw_init); // save imu before return;
    return;
  }

  // use imu pre-integration estimation for pose guess
  static bool lastImuPreTransAvailable = false;
  static Eigen::Affine3f lastImuPreTransformation;
  if (cloudInfo.odom_available == true) {
    Eigen::Affine3f transBack = pcl::getTransformation(
        cloudInfo.initial_guess_x, cloudInfo.initial_guess_y,
        cloudInfo.initial_guess_z, cloudInfo.initial_guess_roll,
        cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
    if (lastImuPreTransAvailable == false) {
      lastImuPreTransformation = transBack;
      lastImuPreTransAvailable = true;
    } else {
      Eigen::Affine3f transIncre =
          lastImuPreTransformation.inverse() * transBack;
      Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
      Eigen::Affine3f transFinal = transTobe * transIncre;
      pcl::getTranslationAndEulerAngles(
          transFinal, transformTobeMapped[3], transformTobeMapped[4],
          transformTobeMapped[5], transformTobeMapped[0],
          transformTobeMapped[1], transformTobeMapped[2]);

      lastImuPreTransformation = transBack;

      lastImuTransformation = pcl::getTransformation(
          0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init,
          cloudInfo.imu_yaw_init); // save imu before return;
      return;
    }
  }

  // use imu incremental estimation for pose guess (only rotation)
  if (cloudInfo.imu_available == true) {
    Eigen::Affine3f transBack = pcl::getTransformation(
        0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init,
        cloudInfo.imu_yaw_init);
    Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

    Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
    Eigen::Affine3f transFinal = transTobe * transIncre;
    pcl::getTranslationAndEulerAngles(
        transFinal, transformTobeMapped[3], transformTobeMapped[4],
        transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1],
        transformTobeMapped[2]);

    lastImuTransformation = pcl::getTransformation(
        0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init,
        cloudInfo.imu_yaw_init); // save imu before return;
    return;
  }
}

void MapOptimization::extractForLoopClosure() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudToExtract(
      new pcl::PointCloud<pcl::PointXYZI>());
  int numPoses = cloudKeyPoses3D->size();
  for (int i = numPoses - 1; i >= 0; --i) {
    if ((int)cloudToExtract->size() <= params_->surrounding_keyframe_size)
      cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
    else
      break;
  }

  extractCloud(cloudToExtract);
}

void MapOptimization::extractNearby() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr surroundingKeyPoses(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surroundingKeyPosesDS(
      new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  // extract all the nearby key poses and downsample them
  kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
  kdtreeSurroundingKeyPoses->radiusSearch(
      cloudKeyPoses3D->back(),
      (double)params_->surrounding_keyframe_search_radius, pointSearchInd,
      pointSearchSqDis);
  for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
    int id = pointSearchInd[i];
    surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
  }

  downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
  downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
  for (auto &pt : surroundingKeyPosesDS->points) {
    kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd,
                                              pointSearchSqDis);
    pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
  }

  // also extract some latest key frames in case the robot rotates in one
  // position
  int numPoses = cloudKeyPoses3D->size();
  for (int i = numPoses - 1; i >= 0; --i) {
    if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
      surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
    else
      break;
  }

  extractCloud(surroundingKeyPosesDS);
}

void MapOptimization::extractCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudToExtract) {
  // fuse the map
  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
    if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) >
        params_->surrounding_keyframe_search_radius)
      continue;

    int thisKeyInd = (int)cloudToExtract->points[i].intensity;
    if (laserCloudMapContainer.find(thisKeyInd) !=
        laserCloudMapContainer.end()) {
      // transformed cloud available
      *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
      *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
    } else {
      // transformed cloud not available
      pcl::PointCloud<pcl::PointXYZI> laserCloudCornerTemp =
          *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                               &cloudKeyPoses6D->points[thisKeyInd]);
      pcl::PointCloud<pcl::PointXYZI> laserCloudSurfTemp = *transformPointCloud(
          surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
      *laserCloudCornerFromMap += laserCloudCornerTemp;
      *laserCloudSurfFromMap += laserCloudSurfTemp;
      laserCloudMapContainer[thisKeyInd] =
          make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
    }
  }

  // Downsample the surrounding corner key frames (or map)
  downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
  // Downsample the surrounding surf key frames (or map)
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

  // clear map cache if too large
  if (laserCloudMapContainer.size() > 1000)
    laserCloudMapContainer.clear();
}

void MapOptimization::extractSurroundingKeyFrames() {
  if (cloudKeyPoses3D->points.empty() == true)
    return;

  // if (params_->loop_closure_enable_flag == true)
  // {
  //     extractForLoopClosure();
  // } else {
  //     extractNearby();
  // }

  extractNearby();
}

void MapOptimization::downsampleCurrentScan() {
  // Downsample cloud from current scan
  laserCloudCornerLastDS->clear();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

  laserCloudSurfLastDS->clear();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void MapOptimization::updatePointAssociateToMap() {
  transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

void MapOptimization::cornerOptimization() {
  updatePointAssociateToMap();

#pragma omp parallel for num_threads(params_->number_of_cores)
  for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
    pcl::PointXYZI pointOri, pointSel, coeff;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = laserCloudCornerLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                        pointSearchSqDis);

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    if (pointSearchSqDis[4] < 1.0) {
      float cx = 0, cy = 0, cz = 0;
      for (int j = 0; j < 5; j++) {
        cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
        cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
        cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int j = 0; j < 5; j++) {
        float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
        float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
        float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      matA1.at<float>(0, 0) = a11;
      matA1.at<float>(0, 1) = a12;
      matA1.at<float>(0, 2) = a13;
      matA1.at<float>(1, 0) = a12;
      matA1.at<float>(1, 1) = a22;
      matA1.at<float>(1, 2) = a23;
      matA1.at<float>(2, 0) = a13;
      matA1.at<float>(2, 1) = a23;
      matA1.at<float>(2, 2) = a33;

      cv::eigen(matA1, matD1, matV1);

      if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = cx + 0.1 * matV1.at<float>(0, 0);
        float y1 = cy + 0.1 * matV1.at<float>(0, 1);
        float z1 = cz + 0.1 * matV1.at<float>(0, 2);
        float x2 = cx - 0.1 * matV1.at<float>(0, 0);
        float y2 = cy - 0.1 * matV1.at<float>(0, 1);
        float z2 = cz - 0.1 * matV1.at<float>(0, 2);

        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                          ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                          ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        float la =
            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
            a012 / l12;

        float lb =
            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float lc =
            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        if (s > 0.1) {
          laserCloudOriCornerVec[i] = pointOri;
          coeffSelCornerVec[i] = coeff;
          laserCloudOriCornerFlag[i] = true;
        }
      }
    }
  }
}

void MapOptimization::surfOptimization() {
  updatePointAssociateToMap();

#pragma omp parallel for num_threads(params_->number_of_cores)
  for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
    pcl::PointXYZI pointOri, pointSel, coeff;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pointOri = laserCloudSurfLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                      pointSearchSqDis);

    Eigen::Matrix<float, 5, 3> matA0;
    Eigen::Matrix<float, 5, 1> matB0;
    Eigen::Vector3f matX0;

    matA0.setZero();
    matB0.fill(-1);
    matX0.setZero();

    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
        matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
        matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
      }

      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                 pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                 pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                 pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointOri.x * pointOri.x +
                                    pointOri.y * pointOri.y +
                                    pointOri.z * pointOri.z));

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        if (s > 0.1) {
          laserCloudOriSurfVec[i] = pointOri;
          coeffSelSurfVec[i] = coeff;
          laserCloudOriSurfFlag[i] = true;
        }
      }
    }
  }
}

void MapOptimization::combineOptimizationCoeffs() {
  // combine corner coeffs
  for (int i = 0; i < laserCloudCornerLastDSNum; ++i) {
    if (laserCloudOriCornerFlag[i] == true) {
      laserCloudOri->push_back(laserCloudOriCornerVec[i]);
      coeffSel->push_back(coeffSelCornerVec[i]);
    }
  }
  // combine surf coeffs
  for (int i = 0; i < laserCloudSurfLastDSNum; ++i) {
    if (laserCloudOriSurfFlag[i] == true) {
      laserCloudOri->push_back(laserCloudOriSurfVec[i]);
      coeffSel->push_back(coeffSelSurfVec[i]);
    }
  }
  // reset flag for next iteration
  std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
            false);
  std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool MapOptimization::LMOptimization(int iterCount) {
  // This optimization is from the original loam_velodyne by Ji Zhang, need to
  // cope with coordinate transformation lidar <- camera      ---     camera <-
  // lidar x = z                ---     x = y y = x                ---     y = z
  // z = y                ---     z = x
  // roll = yaw           ---     roll = pitch
  // pitch = roll         ---     pitch = yaw
  // yaw = pitch          ---     yaw = roll

  // lidar -> camera
  float srx = sin(transformTobeMapped[1]);
  float crx = cos(transformTobeMapped[1]);
  float sry = sin(transformTobeMapped[2]);
  float cry = cos(transformTobeMapped[2]);
  float srz = sin(transformTobeMapped[0]);
  float crz = cos(transformTobeMapped[0]);

  int laserCloudSelNum = laserCloudOri->size();
  if (laserCloudSelNum < 50) {
    return false;
  }

  cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  pcl::PointXYZI pointOri, coeff;

  for (int i = 0; i < laserCloudSelNum; i++) {
    // lidar -> camera
    pointOri.x = laserCloudOri->points[i].y;
    pointOri.y = laserCloudOri->points[i].z;
    pointOri.z = laserCloudOri->points[i].x;
    // lidar -> camera
    coeff.x = coeffSel->points[i].y;
    coeff.y = coeffSel->points[i].z;
    coeff.z = coeffSel->points[i].x;
    coeff.intensity = coeffSel->points[i].intensity;
    // in camera
    float arx =
        (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
         srx * sry * pointOri.z) *
            coeff.x +
        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) *
            coeff.y +
        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
         cry * srx * pointOri.z) *
            coeff.z;

    float ary =
        ((cry * srx * srz - crz * sry) * pointOri.x +
         (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
            coeff.x +
        ((-cry * crz - srx * sry * srz) * pointOri.x +
         (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
            coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                 (-cry * crz - srx * sry * srz) * pointOri.y) *
                    coeff.x +
                (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * pointOri.x +
                 (crz * sry - cry * srx * srz) * pointOri.y) *
                    coeff.z;
    // lidar -> camera
    matA.at<float>(i, 0) = arz;
    matA.at<float>(i, 1) = arx;
    matA.at<float>(i, 2) = ary;
    matA.at<float>(i, 3) = coeff.z;
    matA.at<float>(i, 4) = coeff.x;
    matA.at<float>(i, 5) = coeff.y;
    matB.at<float>(i, 0) = -coeff.intensity;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformTobeMapped[0] += matX.at<float>(0, 0);
  transformTobeMapped[1] += matX.at<float>(1, 0);
  transformTobeMapped[2] += matX.at<float>(2, 0);
  transformTobeMapped[3] += matX.at<float>(3, 0);
  transformTobeMapped[4] += matX.at<float>(4, 0);
  transformTobeMapped[5] += matX.at<float>(5, 0);

  float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                      pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                      pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
  float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                      pow(matX.at<float>(4, 0) * 100, 2) +
                      pow(matX.at<float>(5, 0) * 100, 2));

  if (deltaR < 0.05 && deltaT < 0.05) {
    return true; // converged
  }
  return false; // keep optimizing
}

void MapOptimization::scan2MapOptimization() {
  if (cloudKeyPoses3D->points.empty())
    return;

  if (laserCloudCornerFromMapDS->empty() || laserCloudSurfFromMapDS->empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "No map features available for optimization!");
    return;
  }

  if (laserCloudCornerLastDSNum > params_->valid_edge_feature_num &&
      laserCloudSurfLastDSNum > params_->valid_surf_feature_num) {
    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

    for (int iterCount = 0; iterCount < 30; iterCount++) {
      laserCloudOri->clear();
      coeffSel->clear();

      cornerOptimization();
      surfOptimization();

      combineOptimizationCoeffs();

      if (LMOptimization(iterCount) == true)
        break;
    }

    transformUpdate();
  } else {
    RCLCPP_WARN(
        node_->get_logger(),
        "Not enough features! Only %d edge and %d planar features available.",
        laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
  }
}

void MapOptimization::transformUpdate() {
  if (cloudInfo.imu_available == true) {
    if (std::abs(cloudInfo.imu_pitch_init) < 1.4) {
      double imuWeight = params_->imu_rpy_weight;
      tf2::Quaternion imuQuaternion;
      tf2::Quaternion transformQuaternion;
      double rollMid, pitchMid, yawMid;

      // slerp roll
      transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
      imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
      tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
          .getRPY(rollMid, pitchMid, yawMid);
      transformTobeMapped[0] = rollMid;

      // slerp pitch
      transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
      imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
      tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
          .getRPY(rollMid, pitchMid, yawMid);
      transformTobeMapped[1] = pitchMid;
    }
  }

  transformTobeMapped[0] = constraintTransformation(
      transformTobeMapped[0], params_->rotation_tolerance);
  transformTobeMapped[1] = constraintTransformation(
      transformTobeMapped[1], params_->rotation_tolerance);
  transformTobeMapped[5] =
      constraintTransformation(transformTobeMapped[5], params_->z_tolerance);

  incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

float MapOptimization::constraintTransformation(float value, float limit) {
  if (value < -limit)
    value = -limit;
  if (value > limit)
    value = limit;

  return value;
}

bool MapOptimization::saveFrame() {
  if (cloudKeyPoses3D->points.empty())
    return true;

  if (sensor_ == SensorType::LIVOX) {
    if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
      return true;
  }

  Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
  Eigen::Affine3f transFinal = pcl::getTransformation(
      transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
      transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
  Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

  if (abs(roll) < params_->surrounding_keyframe_angle_threshold &&
      abs(pitch) < params_->surrounding_keyframe_angle_threshold &&
      abs(yaw) < params_->surrounding_keyframe_angle_threshold &&
      sqrt(x * x + y * y + z * z) <
          params_->surrounding_keyframe_distance_threshold)
    return false;

  return true;
}

void MapOptimization::addOdomFactor() {
  if (cloudKeyPoses3D->points.empty()) {
    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                .finished()); // rad*rad, meter*meter
    gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped),
                                      priorNoise));
    initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
  } else {
    noiseModel::Diagonal::shared_ptr odometryNoise =
        noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    gtsam::Pose3 poseFrom =
        pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
    gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
    gtSAMgraph.add(BetweenFactor<Pose3>(
        cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
        poseFrom.between(poseTo), odometryNoise));
    initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
  }
}

void MapOptimization::addGPSFactor() {
  if (gpsQueue.empty())
    return;

  // wait for system initialized and settles down
  if (cloudKeyPoses3D->points.empty())
    return;
  else {
    if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
      return;
  }

  // pose covariance small, no need to correct
  if (poseCovariance(3, 3) < params_->pose_covariance_threshold &&
      poseCovariance(4, 4) < params_->pose_covariance_threshold)
    return;

  // last gps position
  static pcl::PointXYZI lastGPSPoint;

  while (!gpsQueue.empty()) {
    if (stamp2Sec(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2) {
      // message too old
      gpsQueue.pop_front();
    } else if (stamp2Sec(gpsQueue.front().header.stamp) >
               timeLaserInfoCur + 0.2) {
      // message too new
      break;
    } else {
      nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
      gpsQueue.pop_front();

      // GPS too noisy, skip
      float noise_x = thisGPS.pose.covariance[0];
      float noise_y = thisGPS.pose.covariance[7];
      float noise_z = thisGPS.pose.covariance[14];
      if (noise_x > params_->gps_covariance_threshold ||
          noise_y > params_->gps_covariance_threshold)
        continue;
      float gps_x = thisGPS.pose.pose.position.x;
      float gps_y = thisGPS.pose.pose.position.y;
      float gps_z = thisGPS.pose.pose.position.z;
      if (!params_->use_gps_elevation) {
        gps_z = transformTobeMapped[5];
        noise_z = 0.01;
      }

      // GPS not properly initialized (0,0,0)
      if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
        continue;

      // Add GPS every a few meters
      pcl::PointXYZI curGPSPoint;
      curGPSPoint.x = gps_x;
      curGPSPoint.y = gps_y;
      curGPSPoint.z = gps_z;
      if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
        continue;
      else
        lastGPSPoint = curGPSPoint;

      gtsam::Vector Vector3(3);
      Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
      noiseModel::Diagonal::shared_ptr gps_noise =
          noiseModel::Diagonal::Variances(Vector3);
      gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(),
                                  gtsam::Point3(gps_x, gps_y, gps_z),
                                  gps_noise);
      gtSAMgraph.add(gps_factor);

      aLoopIsClosed = true;
      break;
    }
  }
}

void MapOptimization::addLoopFactor() {
  if (loopIndexQueue.empty())
    return;

  for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
    int indexFrom = loopIndexQueue[i].first;
    int indexTo = loopIndexQueue[i].second;
    gtsam::Pose3 poseBetween = loopPoseQueue[i];
    gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
    gtSAMgraph.add(
        BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
  }

  loopIndexQueue.clear();
  loopPoseQueue.clear();
  loopNoiseQueue.clear();
  aLoopIsClosed = true;
}

void MapOptimization::saveKeyFramesAndFactor() {
  if (saveFrame() == false)
    return;

  // odom factor
  addOdomFactor();

  // gps factor
  addGPSFactor();

  // loop factor
  addLoopFactor();

  // cout << "****************************************************" << endl;
  // gtSAMgraph.print("GTSAM Graph:\n");

  // update iSAM
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  if (aLoopIsClosed == true) {
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
  }

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  // save key poses
  pcl::PointXYZI thisPose3D;
  PointTypePose thisPose6D;
  Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate =
      isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
  // cout << "****************************************************" << endl;
  // isamCurrentEstimate.print("Current estimate: ");

  thisPose3D.x = latestEstimate.translation().x();
  thisPose3D.y = latestEstimate.translation().y();
  thisPose3D.z = latestEstimate.translation().z();
  thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
  thisPose6D.roll = latestEstimate.rotation().roll();
  thisPose6D.pitch = latestEstimate.rotation().pitch();
  thisPose6D.yaw = latestEstimate.rotation().yaw();
  thisPose6D.time = timeLaserInfoCur;
  cloudKeyPoses6D->push_back(thisPose6D);

  // cout << "****************************************************" << endl;
  // cout << "Pose covariance:" << endl;
  // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl <<
  // endl;
  poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

  // save updated transform
  transformTobeMapped[0] = latestEstimate.rotation().roll();
  transformTobeMapped[1] = latestEstimate.rotation().pitch();
  transformTobeMapped[2] = latestEstimate.rotation().yaw();
  transformTobeMapped[3] = latestEstimate.translation().x();
  transformTobeMapped[4] = latestEstimate.translation().y();
  transformTobeMapped[5] = latestEstimate.translation().z();

  // save all the received edge and surf points
  pcl::PointCloud<pcl::PointXYZI>::Ptr thisCornerKeyFrame(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr thisSurfKeyFrame(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

  // save key frame cloud
  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);

  // save path for visualization
  updatePath(thisPose6D);
}

void MapOptimization::correctPoses() {
  if (cloudKeyPoses3D->points.empty())
    return;

  if (aLoopIsClosed == true) {
    // clear map cache
    laserCloudMapContainer.clear();
    // clear path
    globalPath.poses.clear();
    // update key poses
    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
      cloudKeyPoses3D->points[i].x =
          isamCurrentEstimate.at<Pose3>(i).translation().x();
      cloudKeyPoses3D->points[i].y =
          isamCurrentEstimate.at<Pose3>(i).translation().y();
      cloudKeyPoses3D->points[i].z =
          isamCurrentEstimate.at<Pose3>(i).translation().z();

      cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
      cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
      cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
      cloudKeyPoses6D->points[i].roll =
          isamCurrentEstimate.at<Pose3>(i).rotation().roll();
      cloudKeyPoses6D->points[i].pitch =
          isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
      cloudKeyPoses6D->points[i].yaw =
          isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

      updatePath(cloudKeyPoses6D->points[i]);
    }

    aLoopIsClosed = false;
  }
}

void MapOptimization::updatePath(const PointTypePose &pose_in) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
  pose_stamped.header.frame_id = params_->odometry_frame;
  pose_stamped.pose.position.x = pose_in.x;
  pose_stamped.pose.position.y = pose_in.y;
  pose_stamped.pose.position.z = pose_in.z;
  tf2::Quaternion q;
  q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();

  globalPath.poses.push_back(pose_stamped);
}

void MapOptimization::publishOdometry() {
  // Publish odometry for ROS (global)
  nav_msgs::msg::Odometry laserOdometryROS;
  laserOdometryROS.header.stamp = timeLaserInfoStamp;
  laserOdometryROS.header.frame_id = params_->odometry_frame;
  laserOdometryROS.child_frame_id = "odom_mapping";
  laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
  laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
  laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1],
                 transformTobeMapped[2]);
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::convert(quat_tf, quat_msg);
  laserOdometryROS.pose.pose.orientation = quat_msg;
  pubLaserOdometryGlobal->publish(laserOdometryROS);

  // Publish TF
  quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1],
                 transformTobeMapped[2]);
  tf2::Transform t_odom_to_lidar = tf2::Transform(
      quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4],
                            transformTobeMapped[5]));
  tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
  tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point,
                                                  params_->odometry_frame);
  geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
  tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
  trans_odom_to_lidar.child_frame_id = "lidar_link";
  br->sendTransform(trans_odom_to_lidar);

  // Publish odometry for ROS (incremental)
  static bool lastIncreOdomPubFlag = false;
  static nav_msgs::msg::Odometry
      laserOdomIncremental;               // incremental odometry msg
  static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
  if (lastIncreOdomPubFlag == false) {
    lastIncreOdomPubFlag = true;
    laserOdomIncremental = laserOdometryROS;
    increOdomAffine = trans2Affine3f(transformTobeMapped);
  } else {
    Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() *
                                  incrementalOdometryAffineBack;
    increOdomAffine = increOdomAffine * affineIncre;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch,
                                      yaw);
    if (cloudInfo.imu_available == true) {
      if (std::abs(cloudInfo.imu_pitch_init) < 1.4) {
        double imuWeight = 0.1;
        tf2::Quaternion imuQuaternion;
        tf2::Quaternion transformQuaternion;
        double rollMid, pitchMid, yawMid;

        // slerp roll
        transformQuaternion.setRPY(roll, 0, 0);
        imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
        tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
            .getRPY(rollMid, pitchMid, yawMid);
        roll = rollMid;

        // slerp pitch
        transformQuaternion.setRPY(0, pitch, 0);
        imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
        tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
            .getRPY(rollMid, pitchMid, yawMid);
        pitch = pitchMid;
      }
    }
    laserOdomIncremental.header.stamp = timeLaserInfoStamp;
    laserOdomIncremental.header.frame_id = params_->odometry_frame;
    laserOdomIncremental.child_frame_id = "odom_mapping";
    laserOdomIncremental.pose.pose.position.x = x;
    laserOdomIncremental.pose.pose.position.y = y;
    laserOdomIncremental.pose.pose.position.z = z;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    laserOdomIncremental.pose.pose.orientation = quat_msg;
    if (isDegenerate)
      laserOdomIncremental.pose.covariance[0] = 1;
    else
      laserOdomIncremental.pose.covariance[0] = 0;
  }
  pubLaserOdometryIncremental->publish(laserOdomIncremental);
}

void MapOptimization::publishFrames() {
  if (cloudKeyPoses3D->points.empty())
    return;
  // publish key poses
  publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp,
               params_->odometry_frame);
  // Publish surrounding key frames
  publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp,
               params_->odometry_frame);
  // publish registered key frame
  if (pubRecentKeyFrame->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
        new pcl::PointCloud<pcl::PointXYZI>());
    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
    *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
    publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp,
                 params_->odometry_frame);
  }
  // publish registered high-res raw cloud
  if (pubCloudRegisteredRaw->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
    publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp,
                 params_->odometry_frame);
  }
  // publish path
  if (pubPath->get_subscription_count() != 0) {
    globalPath.header.stamp = timeLaserInfoStamp;
    globalPath.header.frame_id = params_->odometry_frame;
    pubPath->publish(globalPath);
  }
}
// =============================================================================
// Global Localization Methods
// =============================================================================

void MapOptimization::publishMapToOdomTf(float x, float y, float z, float roll,
                                         float pitch, float yaw,
                                         const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = stamp;
  ts.header.frame_id = params_->map_frame;
  ts.child_frame_id = params_->odometry_frame;
  ts.transform.translation.x = x;
  ts.transform.translation.y = y;
  ts.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  mapToOdomBroadcaster->sendTransform(ts);
}

void MapOptimization::cloudGlobalLoad() {
  if (!params_->load_map_file_dir.empty() &&
      params_->load_map_file_dir.back() != '/')
    params_->load_map_file_dir += "/";
  pcl::io::loadPCDFile<pcl::PointXYZI>(
      params_->load_map_file_dir + "GlobalMap.pcd", *cloudGlobalMap);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(
      new pcl::PointCloud<pcl::PointXYZI>());
  downSizeFilterICP.setInputCloud(cloudGlobalMap);
  downSizeFilterICP.filter(*cloud_temp);
  *cloudGlobalMapDS = *cloud_temp;

  RCLCPP_INFO(node_->get_logger(), "Global map loaded with %zu points.",
              cloudGlobalMap->size());
  RCLCPP_INFO(node_->get_logger(), "Global map downsampled to %zu points.",
              cloudGlobalMapDS->size());

  sleep(3);
  publishCloud(pubGlobalMap, cloudGlobalMapDS, rclcpp::Time(),
               params_->map_frame);
}

void MapOptimization::globalLocalizeThreadFunc() {
  while (rclcpp::ok()) {
    if (initializedFlag == NonInitialized) {
      ICPLocalizeInitialize();
    } else if (initializedFlag == Initializing) {
      RCLCPP_INFO(node_->get_logger(), "Offer A new Guess Please!");
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      rclcpp::sleep_for(std::chrono::seconds(10));
      ICPscanMatchGlobal();
    }
  }
}

void MapOptimization::ICPLocalizeInitialize() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(
      new pcl::PointCloud<pcl::PointXYZI>());

  mtx_general.lock();
  *laserCloudIn += *cloudScanForInitialize;
  mtx_general.unlock();

  if (laserCloudIn->points.size() == 0) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "ICPLocalizeInitialize: incoming cloud size = %zu",
              laserCloudIn->points.size());

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setResolution(1.0);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  ndt.setInputSource(laserCloudIn);
  ndt.setInputTarget(cloudGlobalMapDS);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result_0(
      new pcl::PointCloud<pcl::PointXYZI>());

  PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
  Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
  ndt.align(*unused_result_0, T_thisPose6DInWorld.matrix());

  icp.setInputSource(laserCloudIn);
  icp.setInputTarget(cloudGlobalMapDS);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(
      new pcl::PointCloud<pcl::PointXYZI>());
  icp.align(*unused_result, ndt.getFinalTransformation());

  RCLCPP_INFO(node_->get_logger(),
              "ICP init score: %.4f, converged: %d",
              icp.getFitnessScore(), icp.hasConverged());

  PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
  Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

  Eigen::Affine3f T_thisPose6DInMap;
  T_thisPose6DInMap = icp.getFinalTransformation();
  float x_g, y_g, z_g, R_g, P_g, Y_g;
  pcl::getTranslationAndEulerAngles(T_thisPose6DInMap, x_g, y_g, z_g, R_g,
                                    P_g, Y_g);
  transformInTheWorld[0] = R_g;
  transformInTheWorld[1] = P_g;
  transformInTheWorld[2] = Y_g;
  transformInTheWorld[3] = x_g;
  transformInTheWorld[4] = y_g;
  transformInTheWorld[5] = z_g;

  Eigen::Affine3f transOdomToMap =
      T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
  float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
  pcl::getTranslationAndEulerAngles(transOdomToMap, deltax, deltay, deltaz,
                                    deltaR, deltaP, deltaY);

  mtxtransformOdomToWorld.lock();
  transformOdomToWorld[0] = deltaR;
  transformOdomToWorld[1] = deltaP;
  transformOdomToWorld[2] = deltaY;
  transformOdomToWorld[3] = deltax;
  transformOdomToWorld[4] = deltay;
  transformOdomToWorld[5] = deltaz;
  mtxtransformOdomToWorld.unlock();

  publishCloud(pubLaserCloudInWorld, unused_result, timeLaserInfoStamp,
               params_->map_frame);
  publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp,
               params_->map_frame);

  if (icp.hasConverged() == false ||
      icp.getFitnessScore() > params_->history_keyframe_fitness_score) {
    initializedFlag = Initializing;
    RCLCPP_WARN(node_->get_logger(), "Initializing Fail");
    return;
  } else {
    initializedFlag = Initialized;
    RCLCPP_INFO(node_->get_logger(), "Initializing Succeed");
    geometry_msgs::msg::PoseStamped pose_odomTo_map;
    tf2::Quaternion q_odomTo_map;
    q_odomTo_map.setRPY(deltaR, deltaP, deltaY);

    pose_odomTo_map.header.stamp = timeLaserInfoStamp;
    pose_odomTo_map.header.frame_id = "map";
    pose_odomTo_map.pose.position.x = deltax;
    pose_odomTo_map.pose.position.y = deltay;
    pose_odomTo_map.pose.position.z = deltaz;
    pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
    pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
    pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
    pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
    pubOdomToMapPose->publish(pose_odomTo_map);
    publishMapToOdomTf(deltax, deltay, deltaz, deltaR, deltaP, deltaY,
                       timeLaserInfoStamp);
  }
}

void MapOptimization::ICPscanMatchGlobal() {
  if (window_cornerCloudKeyFrames.empty() ||
      window_surfCloudKeyFrames.empty() || window_cloudKeyPoses6D.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "ICPscanMatchGlobal: Keyframe vectors are empty!");
    return;
  }

  if (cloudKeyPoses3D->points.empty()) {
    return;
  }

  mtxWindow.lock();
  int latestFrameIDGlobalLocalize =
      window_cornerCloudKeyFrames.size() - 1;
  if (latestFrameIDGlobalLocalize < 0) {
    mtxWindow.unlock();
    RCLCPP_WARN(node_->get_logger(),
                "ICPscanMatchGlobal: No valid keyframe index!");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr latestCloudIn(
      new pcl::PointCloud<pcl::PointXYZI>());
  *latestCloudIn += *transformPointCloud(
      window_cornerCloudKeyFrames[latestFrameIDGlobalLocalize],
      &window_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
  *latestCloudIn += *transformPointCloud(
      window_surfCloudKeyFrames[latestFrameIDGlobalLocalize],
      &window_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
  mtxWindow.unlock();

  if (latestCloudIn->empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "ICPscanMatchGlobal: latestCloudIn is empty!");
    return;
  }

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setResolution(1.0);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  mtxtransformOdomToWorld.lock();
  Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(
      transformOdomToWorld[3], transformOdomToWorld[4],
      transformOdomToWorld[5], transformOdomToWorld[0],
      transformOdomToWorld[1], transformOdomToWorld[2]);
  mtxtransformOdomToWorld.unlock();

  Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();
  ndt.setInputSource(latestCloudIn);
  ndt.setInputTarget(cloudGlobalMapDS);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result_0(
      new pcl::PointCloud<pcl::PointXYZI>());
  ndt.align(*unused_result_0, matricInitGuess);

  icp.setInputSource(latestCloudIn);
  icp.setInputTarget(cloudGlobalMapDS);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(
      new pcl::PointCloud<pcl::PointXYZI>());
  icp.align(*unused_result, ndt.getFinalTransformation());

  RCLCPP_INFO(node_->get_logger(),
              "ICP global match converged: %d, score: %.4f",
              icp.hasConverged(), icp.getFitnessScore());

  Eigen::Affine3f transodomToWorld_New;
  transodomToWorld_New = icp.getFinalTransformation();
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(transodomToWorld_New, x, y, z, roll,
                                    pitch, yaw);

  mtxtransformOdomToWorld.lock();
  transformOdomToWorld[0] = roll;
  transformOdomToWorld[1] = pitch;
  transformOdomToWorld[2] = yaw;
  transformOdomToWorld[3] = x;
  transformOdomToWorld[4] = y;
  transformOdomToWorld[5] = z;
  mtxtransformOdomToWorld.unlock();

  publishCloud(pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp,
               params_->map_frame);

  if (icp.hasConverged() == true &&
      icp.getFitnessScore() < params_->history_keyframe_fitness_score) {
    geometry_msgs::msg::PoseStamped pose_odomTo_map;
    tf2::Quaternion q_odomTo_map;
    q_odomTo_map.setRPY(roll, pitch, yaw);

    pose_odomTo_map.header.stamp = timeLaserInfoStamp;
    pose_odomTo_map.header.frame_id = params_->map_frame;
    pose_odomTo_map.pose.position.x = x;
    pose_odomTo_map.pose.position.y = y;
    pose_odomTo_map.pose.position.z = z;
    pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
    pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
    pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
    pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
    pubOdomToMapPose->publish(pose_odomTo_map);
    publishMapToOdomTf(x, y, z, roll, pitch, yaw, timeLaserInfoStamp);
  }
}

void MapOptimization::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
  if (initializedFlag == Initialized) {
    return;
  }

  float x = pose_msg->pose.pose.position.x;
  float y = pose_msg->pose.pose.position.y;
  float z = pose_msg->pose.pose.position.z;

  tf2::Quaternion q_global;
  double roll_global, pitch_global, yaw_global;

  q_global.setX(pose_msg->pose.pose.orientation.x);
  q_global.setY(pose_msg->pose.pose.orientation.y);
  q_global.setZ(pose_msg->pose.pose.orientation.z);
  q_global.setW(pose_msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);

  transformInTheWorld[0] = roll_global;
  transformInTheWorld[1] = pitch_global;
  transformInTheWorld[2] = yaw_global;
  transformInTheWorld[3] = x;
  transformInTheWorld[4] = y;
  transformInTheWorld[5] = z;

  PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
  Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);

  PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
  Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

  Eigen::Affine3f T_OdomToMap =
      T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
  float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
  pcl::getTranslationAndEulerAngles(T_OdomToMap, delta_x, delta_y, delta_z,
                                    delta_roll, delta_pitch, delta_yaw);

  mtxtransformOdomToWorld.lock();
  transformOdomToWorld[0] = delta_roll;
  transformOdomToWorld[1] = delta_pitch;
  transformOdomToWorld[2] = delta_yaw;
  transformOdomToWorld[3] = delta_x;
  transformOdomToWorld[4] = delta_y;
  transformOdomToWorld[5] = delta_z;
  mtxtransformOdomToWorld.unlock();

  initializedFlag = NonInitialized;
}

void MapOptimization::extractForLocalization() {
  if (cloudKeyPoses3D->points.empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudToExtract(
      new pcl::PointCloud<pcl::PointXYZI>());
  int numPoses = window_cloudKeyPoses3D.size();
  for (int i = numPoses - 1; i >= 0; --i) {
    cloudToExtract->push_back(window_cloudKeyPoses3D[i]);
  }

  std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudCornerSurroundingVec;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudSurfSurroundingVec;

  laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
  laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

  #pragma omp parallel for num_threads(params_->number_of_cores)
  for (int i = 0; i < static_cast<int>(cloudToExtract->size()); ++i) {
    PointTypePose thisPose6D = window_cloudKeyPoses6D[i];
    laserCloudCornerSurroundingVec[i] =
        *transformPointCloud(window_cornerCloudKeyFrames[i], &thisPose6D);
    laserCloudSurfSurroundingVec[i] =
        *transformPointCloud(window_surfCloudKeyFrames[i], &thisPose6D);
  }

  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  for (int i = 0; i < static_cast<int>(cloudToExtract->size()); ++i) {
    *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
    *laserCloudSurfFromMap += laserCloudSurfSurroundingVec[i];
  }

  downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
}

void MapOptimization::saveKeyFramesAndFactorLocalization() {
  if (saveFrame() == false) {
    return;
  }

  addOdomFactor();

  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  pcl::PointXYZI thisPose3D;
  PointTypePose thisPose6D;
  gtsam::Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(
      isamCurrentEstimate.size() - 1);

  thisPose3D.x = latestEstimate.translation().x();
  thisPose3D.y = latestEstimate.translation().y();
  thisPose3D.z = latestEstimate.translation().z();
  thisPose3D.intensity = cloudKeyPoses3D->size();
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity;
  thisPose6D.roll = latestEstimate.rotation().roll();
  thisPose6D.pitch = latestEstimate.rotation().pitch();
  thisPose6D.yaw = latestEstimate.rotation().yaw();
  thisPose6D.time = timeLaserInfoCur;
  cloudKeyPoses6D->push_back(thisPose6D);

  mtxWindow.lock();

  window_cloudKeyPoses3D.push_back(thisPose3D);
  window_cloudKeyPoses6D.push_back(thisPose6D);
  if (static_cast<int>(window_cloudKeyPoses3D.size()) > winSize) {
    window_cloudKeyPoses3D.erase(window_cloudKeyPoses3D.begin());
    window_cloudKeyPoses6D.erase(window_cloudKeyPoses6D.begin());
  }

  poseCovariance =
      isam->marginalCovariance(isamCurrentEstimate.size() - 1);

  transformTobeMapped[0] = latestEstimate.rotation().roll();
  transformTobeMapped[1] = latestEstimate.rotation().pitch();
  transformTobeMapped[2] = latestEstimate.rotation().yaw();
  transformTobeMapped[3] = latestEstimate.translation().x();
  transformTobeMapped[4] = latestEstimate.translation().y();
  transformTobeMapped[5] = latestEstimate.translation().z();

  pcl::PointCloud<pcl::PointXYZI>::Ptr thisCornerKeyFrame(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr thisSurfKeyFrame(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);

  window_cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  window_surfCloudKeyFrames.push_back(thisSurfKeyFrame);
  if (static_cast<int>(window_cornerCloudKeyFrames.size()) > winSize) {
    window_cornerCloudKeyFrames.erase(window_cornerCloudKeyFrames.begin());
    window_surfCloudKeyFrames.erase(window_surfCloudKeyFrames.begin());
  }
  mtxWindow.unlock();

  PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
  mtxtransformOdomToWorld.lock();
  PointTypePose pose_Odom_Map = trans2PointTypePose(transformOdomToWorld);
  mtxtransformOdomToWorld.unlock();

  Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
  Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);
  Eigen::Affine3f T_thisPose6DInMap = T_pose_Odom_Map * T_thisPose6DInOdom;

  float x_map, y_map, z_map, roll_map, pitch_map, yaw_map;
  pcl::getTranslationAndEulerAngles(T_thisPose6DInMap, x_map, y_map, z_map,
                                    roll_map, pitch_map, yaw_map);

  PointTypePose thisPose6DInMap;
  thisPose6DInMap.x = x_map;
  thisPose6DInMap.y = y_map;
  thisPose6DInMap.z = z_map;
  thisPose6DInMap.roll = roll_map;
  thisPose6DInMap.pitch = pitch_map;
  thisPose6DInMap.yaw = yaw_map;
  thisPose6DInMap.intensity = thisPose6D.intensity;
  thisPose6DInMap.time = thisPose6D.time;

  updatePath(thisPose6DInMap);
}

void MapOptimization::publishFramesLocalization() {
  if (cloudKeyPoses3D->points.empty()) {
    return;
  }

  publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp,
               params_->odometry_frame);
  publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS,
               timeLaserInfoStamp, params_->odometry_frame);

  if (pubRecentKeyFrame->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
        new pcl::PointCloud<pcl::PointXYZI>());
    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    Eigen::Affine3f T_thisPose6D = pclPointToAffine3f(thisPose6D);

    mtxtransformOdomToWorld.lock();
    PointTypePose pose_Odom_Map = trans2PointTypePose(transformOdomToWorld);
    mtxtransformOdomToWorld.unlock();
    Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);
    Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6D;

    float x_map, y_map, z_map, roll_map, pitch_map, yaw_map;
    pcl::getTranslationAndEulerAngles(T_poseInMap, x_map, y_map, z_map,
                                      roll_map, pitch_map, yaw_map);

    PointTypePose thisPose6DInMap;
    thisPose6DInMap.x = x_map;
    thisPose6DInMap.y = y_map;
    thisPose6DInMap.z = z_map;
    thisPose6DInMap.roll = roll_map;
    thisPose6DInMap.pitch = pitch_map;
    thisPose6DInMap.yaw = yaw_map;
    thisPose6DInMap.intensity = thisPose6D.intensity;
    thisPose6DInMap.time = thisPose6D.time;

    *cloudOut += *transformPointCloud(laserCloudCornerLast, &thisPose6DInMap);
    *cloudOut += *transformPointCloud(laserCloudSurfLast, &thisPose6DInMap);
    publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp,
                 params_->odometry_frame);
  }

  if (pubLaserCloudInWorld->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInBase(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutInWorld(
        new pcl::PointCloud<pcl::PointXYZI>());
    PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
    Eigen::Affine3f T_thisPose6DInOdom =
        pclPointToAffine3f(thisPose6DInOdom);
    mtxtransformOdomToWorld.lock();
    PointTypePose pose_Odom_Map = trans2PointTypePose(transformOdomToWorld);
    mtxtransformOdomToWorld.unlock();
    Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);

    Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6DInOdom;
    *cloudInBase += *laserCloudCornerLastDS;
    *cloudInBase += *laserCloudSurfLastDS;
    pcl::transformPointCloud(*cloudInBase, *cloudOutInWorld,
                             T_poseInMap.matrix());
    publishCloud(pubLaserCloudInWorld, cloudOutInWorld, timeLaserInfoStamp,
                 params_->map_frame);
  }

  if (pubCloudRegisteredRaw->get_subscription_count() != 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    Eigen::Affine3f T_thisPose6D = pclPointToAffine3f(thisPose6D);

    mtxtransformOdomToWorld.lock();
    PointTypePose pose_Odom_Map = trans2PointTypePose(transformOdomToWorld);
    mtxtransformOdomToWorld.unlock();
    Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);
    Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6D;

    float x_map, y_map, z_map, roll_map, pitch_map, yaw_map;
    pcl::getTranslationAndEulerAngles(T_poseInMap, x_map, y_map, z_map,
                                      roll_map, pitch_map, yaw_map);

    PointTypePose thisPose6DInMap;
    thisPose6DInMap.x = x_map;
    thisPose6DInMap.y = y_map;
    thisPose6DInMap.z = z_map;
    thisPose6DInMap.roll = roll_map;
    thisPose6DInMap.pitch = pitch_map;
    thisPose6DInMap.yaw = yaw_map;
    thisPose6DInMap.intensity = thisPose6D.intensity;
    thisPose6DInMap.time = thisPose6D.time;

    *cloudOut = *transformPointCloud(cloudOut, &thisPose6DInMap);
    publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp,
                 params_->odometry_frame);
  }

  if (pubPath->get_subscription_count() != 0) {
    globalPath.header.stamp = timeLaserInfoStamp;
    globalPath.header.frame_id = params_->odometry_frame;
    pubPath->publish(globalPath);
  }
}

} //  namespace lio_sam

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);

//     rclcpp::NodeOptions options;
//     options.use_intra_process_comms(true);
//     rclcpp::executors::SingleThreadedExecutor exec;

//     auto MO = std::make_shared<lio_sam::MapOptimization>(options);
//     exec.add_node(MO);

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map
//     Optimization Started.\033[0m");

//     std::thread loopthread(&lio_sam::MapOptimization::loopClosureThread, MO);
//     std::thread
//     visualizeMapThread(&lio_sam::MapOptimization::visualizeGlobalMapThread,
//     MO);

//     exec.spin();

//     rclcpp::shutdown();

//     loopthread.join();
//     visualizeMapThread.join();

//     return 0;
// }
