#include "lio_sam/image_projection.hpp"

namespace lio_sam {
ImageProjection::ImageProjection(rclcpp_lifecycle::LifecycleNode *node_,
                                 std::shared_ptr<LioSamParams> params)
    : node_(node_), params_(params), deskewFlag(0) {
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

  callbackGroupLidar = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackGroupImu = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackGroupOdom = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  auto lidarOpt = rclcpp::SubscriptionOptions();
  lidarOpt.callback_group = callbackGroupLidar;
  auto imuOpt = rclcpp::SubscriptionOptions();
  imuOpt.callback_group = callbackGroupImu;
  auto odomOpt = rclcpp::SubscriptionOptions();
  odomOpt.callback_group = callbackGroupOdom;

  subImu = node_->create_subscription<sensor_msgs::msg::Imu>(
      params_->imu_topic, qos_imu,
      std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
      imuOpt);
  subOdom = node_->create_subscription<nav_msgs::msg::Odometry>(
      params_->odom_topic + "_incremental", qos_imu,
      std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
      odomOpt);
  subLaserCloud = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_->point_cloud_topic, qos_lidar,
      std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
      lidarOpt);

  pubExtractedCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lio_sam/deskew/cloud_deskewed", 1);
  pubLaserCloudInfo = node_->create_publisher<lio_sam_with_global_localization::msg::CloudInfo>(
      "lio_sam/deskew/cloud_info", qos);

  allocateMemory();
  resetParameters();

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

ImageProjection::~ImageProjection() {}

void ImageProjection::allocateMemory() {
  laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
  tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
  fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  extractedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  fullCloud->points.resize(params_->vertical_scan_num *
                           params_->horizontal_scan_num);

  cloudInfo.start_ring_index.assign(params_->vertical_scan_num, 0);
  cloudInfo.end_ring_index.assign(params_->vertical_scan_num, 0);

  cloudInfo.point_col_ind.assign(
      params_->vertical_scan_num * params_->horizontal_scan_num, 0);
  cloudInfo.point_range.assign(
      params_->vertical_scan_num * params_->horizontal_scan_num, 0);

  imuTime = new double[queueLength];
  imuRotX = new double[queueLength];
  imuRotY = new double[queueLength];
  imuRotZ = new double[queueLength];

  resetParameters();
}

void ImageProjection::resetParameters() {
  laserCloudIn->clear();
  extractedCloud->clear();
  // reset range matrix for range image projection
  rangeMat = cv::Mat(params_->vertical_scan_num, params_->horizontal_scan_num,
                     CV_32F, cv::Scalar::all(FLT_MAX));

  imuPointerCur = 0;
  firstPointFlag = true;
  odomDeskewFlag = false;

  for (int i = 0; i < queueLength; ++i) {
    imuTime[i] = 0;
    imuRotX[i] = 0;
    imuRotY[i] = 0;
    imuRotZ[i] = 0;
  }
  columnIdnCountVec.assign(params_->vertical_scan_num, 0);
}

void ImageProjection::imuHandler(
    const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
  sensor_msgs::msg::Imu thisImu =
      imuConverter(*imuMsg, params_->extRot, params_->extQRPY);

  std::lock_guard<std::mutex> lock1(imuLock);
  // thisImu.header.stamp.sec -= 3600;
  // thisImu.header.stamp.nanosec -= 118;
  imuQueue.push_back(thisImu);

  // debug IMU data
  // cout << std::setprecision(6);
  // cout << "IMU acc: " << endl;
  // cout << "x: " << thisImu.linear_acceleration.x <<
  //       ", y: " << thisImu.linear_acceleration.y <<
  //       ", z: " << thisImu.linear_acceleration.z << endl;
  // cout << "IMU gyro: " << endl;
  // cout << "x: " << thisImu.angular_velocity.x <<
  //       ", y: " << thisImu.angular_velocity.y <<
  //       ", z: " << thisImu.angular_velocity.z << endl;
  // double imuRoll, imuPitch, imuYaw;
  // tf2::Quaternion orientation;
  // tf2::fromMsg(thisImu.orientation, orientation);
  // tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
  // cout << "IMU roll pitch yaw: " << endl;
  // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " <<
  // imuYaw << endl << endl;
}

void ImageProjection::odometryHandler(
    const nav_msgs::msg::Odometry::SharedPtr odometryMsg) {
  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(*odometryMsg);
}

void ImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
  if (!cachePointCloud(laserCloudMsg))
    return;

  if (!deskewInfo())
    return;

  projectPointCloud();

  cloudExtraction();

  publishClouds();

  resetParameters();
}

bool ImageProjection::cachePointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg) {
  // cache point cloud
  cloudQueue.push_back(*laserCloudMsg);
  if (cloudQueue.size() <= 2)
    return false;

  // convert cloud
  currentCloudMsg = std::move(cloudQueue.front());
  cloudQueue.pop_front();
  if (sensor_ == SensorType::VELODYNE || sensor_ == SensorType::LIVOX) {
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    if (static_cast<int>(laserCloudIn->points.size()) <= 32) {
      return false;
    }
  } else if (sensor_ == SensorType::OUSTER) {
    // Convert to Velodyne format
    pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
    laserCloudIn->points.resize(tmpOusterCloudIn->size());
    laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
    for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
      auto &src = tmpOusterCloudIn->points[i];
      auto &dst = laserCloudIn->points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.intensity = src.intensity;
      dst.ring = src.ring;
      dst.time = src.t * 1e-9f;
    }
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Unknown sensor type: " << int(sensor_));
    rclcpp::shutdown();
  }

  // get timestamp
  cloudHeader = currentCloudMsg.header;
  timeScanCur = stamp2Sec(cloudHeader.stamp);
  timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

  // remove Nan
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

  // check dense flag
  if (laserCloudIn->is_dense == false) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Point cloud is not in dense format, please remove NaN points first!");
    rclcpp::shutdown();
  }

  // check ring channel
  // we will skip the ring check in case of velodyne - as we calculate the ring
  // value downstream (line 572)
  if (ringFlag == 0) {
    ringFlag = -1;
    for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i) {
      if (currentCloudMsg.fields[i].name == "ring") {
        ringFlag = 1;
        break;
      }
    }
    if (ringFlag == -1) {
      if (sensor_ == SensorType::VELODYNE) {
        ringFlag = 2;
      } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "Point cloud ring channel not available, "
                     "please configure your point cloud data!");
        rclcpp::shutdown();
      }
    }
  }

  // check point time
  if (deskewFlag == 0) {
    deskewFlag = -1;
    for (auto &field : currentCloudMsg.fields) {
      if (field.name == "time" || field.name == "t" ||
          field.name == "timestamp") {
        deskewFlag = 1;
        break;
      }
    }
    if (deskewFlag == -1)
      RCLCPP_WARN(node_->get_logger(),
                  "Point cloud timestamp not available, deskew function "
                  "disabled, system will drift significantly!");
  }

  return true;
}

bool ImageProjection::deskewInfo() {
  std::lock_guard<std::mutex> lock1(imuLock);
  std::lock_guard<std::mutex> lock2(odoLock);

  // make sure IMU data available for the scan
  if (imuQueue.empty() ||
      stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
      stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for IMU data ...");
    return false;
  }

  imuDeskewInfo();

  odomDeskewInfo();

  return true;
}

void ImageProjection::imuDeskewInfo() {
  cloudInfo.imu_available = false;

  while (!imuQueue.empty()) {
    if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
      imuQueue.pop_front();
    else
      break;
  }

  if (imuQueue.empty())
    return;

  imuPointerCur = 0;

  for (int i = 0; i < (int)imuQueue.size(); ++i) {
    sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
    double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

    // get roll, pitch, and yaw estimation for this scan
    if (currentImuTime <= timeScanCur)
      imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init,
                    &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
    if (currentImuTime > timeScanEnd + 0.01)
      break;

    if (imuPointerCur == 0) {
      imuRotX[0] = 0;
      imuRotY[0] = 0;
      imuRotZ[0] = 0;
      imuTime[0] = currentImuTime;
      ++imuPointerCur;
      continue;
    }

    // get angular velocity
    double angular_x, angular_y, angular_z;
    imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

    // integrate rotation
    double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
    imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
    imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
    imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
    imuTime[imuPointerCur] = currentImuTime;
    ++imuPointerCur;
  }

  --imuPointerCur;

  if (imuPointerCur <= 0)
    return;

  cloudInfo.imu_available = true;
}

void ImageProjection::odomDeskewInfo() {
  cloudInfo.odom_available = false;

  while (!odomQueue.empty()) {
    if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
      odomQueue.pop_front();
    else
      break;
  }

  if (odomQueue.empty())
    return;

  if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
    return;

  // get start odometry at the beinning of the scan
  nav_msgs::msg::Odometry startOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    startOdomMsg = odomQueue[i];

    if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
      continue;
    else
      break;
  }

  tf2::Quaternion orientation;
  tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // Initial guess used in mapOptimization
  cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
  cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
  cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
  cloudInfo.initial_guess_roll = roll;
  cloudInfo.initial_guess_pitch = pitch;
  cloudInfo.initial_guess_yaw = yaw;

  cloudInfo.odom_available = true;

  // get end odometry at the end of the scan
  odomDeskewFlag = false;

  if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
    return;

  nav_msgs::msg::Odometry endOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    endOdomMsg = odomQueue[i];

    if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
      continue;
    else
      break;
  }

  if (int(round(startOdomMsg.pose.covariance[0])) !=
      int(round(endOdomMsg.pose.covariance[0])))
    return;

  Eigen::Affine3f transBegin = pcl::getTransformation(
      startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
      startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  Eigen::Affine3f transEnd = pcl::getTransformation(
      endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y,
      endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

  float rollIncre, pitchIncre, yawIncre;
  pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ,
                                    rollIncre, pitchIncre, yawIncre);

  odomDeskewFlag = true;
}

void ImageProjection::findRotation(double pointTime, float *rotXCur,
                                   float *rotYCur, float *rotZCur) {
  *rotXCur = 0;
  *rotYCur = 0;
  *rotZCur = 0;

  int imuPointerFront = 0;
  while (imuPointerFront < imuPointerCur) {
    if (pointTime < imuTime[imuPointerFront])
      break;
    ++imuPointerFront;
  }

  if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
    *rotXCur = imuRotX[imuPointerFront];
    *rotYCur = imuRotY[imuPointerFront];
    *rotZCur = imuRotZ[imuPointerFront];
  } else {
    int imuPointerBack = imuPointerFront - 1;
    double ratioFront = (pointTime - imuTime[imuPointerBack]) /
                        (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    double ratioBack = (imuTime[imuPointerFront] - pointTime) /
                       (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    *rotXCur = imuRotX[imuPointerFront] * ratioFront +
               imuRotX[imuPointerBack] * ratioBack;
    *rotYCur = imuRotY[imuPointerFront] * ratioFront +
               imuRotY[imuPointerBack] * ratioBack;
    *rotZCur = imuRotZ[imuPointerFront] * ratioFront +
               imuRotZ[imuPointerBack] * ratioBack;
  }
}

void ImageProjection::findPosition(double relTime, float *posXCur,
                                   float *posYCur, float *posZCur) {
  *posXCur = 0;
  *posYCur = 0;
  *posZCur = 0;

  // If the sensor moves relatively slow, like walking speed, positional deskew
  // seems to have little benefits. Thus code below is commented.

  // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
  //     return;

  // float ratio = relTime / (timeScanEnd - timeScanCur);

  // *posXCur = ratio * odomIncreX;
  // *posYCur = ratio * odomIncreY;
  // *posZCur = ratio * odomIncreZ;
}

pcl::PointXYZI ImageProjection::deskewPoint(pcl::PointXYZI *point,
                                            double relTime) {
  if (deskewFlag == -1 || cloudInfo.imu_available == false)
    return *point;

  double pointTime = timeScanCur + relTime;

  float rotXCur, rotYCur, rotZCur;
  findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

  float posXCur, posYCur, posZCur;
  findPosition(relTime, &posXCur, &posYCur, &posZCur);

  if (firstPointFlag == true) {
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                rotXCur, rotYCur, rotZCur))
                            .inverse();
    firstPointFlag = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(
      posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
  Eigen::Affine3f transBt = transStartInverse * transFinal;

  pcl::PointXYZI newPoint;
  newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y +
               transBt(0, 2) * point->z + transBt(0, 3);
  newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y +
               transBt(1, 2) * point->z + transBt(1, 3);
  newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y +
               transBt(2, 2) * point->z + transBt(2, 3);
  newPoint.intensity = point->intensity;

  return newPoint;
}

void ImageProjection::projectPointCloud() {
  int cloudSize = laserCloudIn->points.size();
  // range image projection
  for (int i = 0; i < cloudSize; ++i) {
    pcl::PointXYZI thisPoint;
    thisPoint.x = laserCloudIn->points[i].x;
    thisPoint.y = laserCloudIn->points[i].y;
    thisPoint.z = laserCloudIn->points[i].z;
    thisPoint.intensity = laserCloudIn->points[i].intensity;

    float range = pointDistance(thisPoint);
    if (range < params_->lidar_min_range || range > params_->lidar_max_range)
      continue;

    int rowIdn = laserCloudIn->points[i].ring;
    // if sensor is a velodyne (ringFlag = 2) calculate rowIdn based on number
    // of scans
    if (ringFlag == 2) {
      float verticalAngle =
          atan2(thisPoint.z,
                sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) *
          180 / M_PI;
      rowIdn = (verticalAngle + (params_->vertical_scan_num - 1)) / 2.0;
    }

    if (rowIdn < 0 || rowIdn >= params_->vertical_scan_num)
      continue;

    if (rowIdn % params_->downsample_rate != 0)
      continue;

    int columnIdn = -1;
    if (sensor_ == SensorType::VELODYNE || sensor_ == SensorType::OUSTER) {
      float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      static float ang_res_x = 360.0 / float(params_->horizontal_scan_num);
      columnIdn = -round((horizonAngle - 90.0) / ang_res_x) +
                  params_->horizontal_scan_num / 2;
      if (columnIdn >= params_->horizontal_scan_num)
        columnIdn -= params_->horizontal_scan_num;
    } else if (sensor_ == SensorType::LIVOX) {
      columnIdn = columnIdnCountVec[rowIdn];
      columnIdnCountVec[rowIdn] += 1;
    }

    if (columnIdn < 0 || columnIdn >= params_->horizontal_scan_num)
      continue;

    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
      continue;

    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

    rangeMat.at<float>(rowIdn, columnIdn) = range;

    int index = columnIdn + rowIdn * params_->horizontal_scan_num;
    fullCloud->points[index] = thisPoint;
  }
}

void ImageProjection::cloudExtraction() {
  int count = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < params_->vertical_scan_num; ++i) {
    cloudInfo.start_ring_index[i] = count - 1 + 5;
    for (int j = 0; j < params_->horizontal_scan_num; ++j) {
      if (rangeMat.at<float>(i, j) != FLT_MAX) {
        // mark the points' column index for marking occlusion later
        cloudInfo.point_col_ind[count] = j;
        // save range info
        cloudInfo.point_range[count] = rangeMat.at<float>(i, j);
        // save extracted cloud
        extractedCloud->push_back(
            fullCloud->points[j + i * params_->horizontal_scan_num]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.end_ring_index[i] = count - 1 - 5;
  }
}

void ImageProjection::publishClouds() {
  cloudInfo.header = cloudHeader;
  cloudInfo.cloud_deskewed =
      publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp,
                   params_->lidar_frame);
  pubLaserCloudInfo->publish(cloudInfo);
}
} // namespace lio_sam

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.use_intra_process_comms(true);
//   rclcpp::executors::MultiThreadedExecutor exec;

//   auto IP = std::make_shared<lio_sam::ImageProjection>(options);
//   exec.add_node(IP);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection
//   Started.\033[0m");

//   exec.spin();

//   rclcpp::shutdown();
//   return 0;
// }
