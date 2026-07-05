#ifndef LIO_SAM_IMU_PREINTEGRATION_HPP_
#define LIO_SAM_IMU_PREINTEGRATION_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lio_sam/utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>


using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

namespace lio_sam
{
class IMUPreintegration
{
public:
  IMUPreintegration(
    rclcpp_lifecycle::LifecycleNode * node_,
    std::shared_ptr<LioSamParams> params);
  virtual ~IMUPreintegration();

private:
  std::mutex mtx;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;

  rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
  rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
  gtsam::Vector noiseModelBetweenBias;


  gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

  std::deque<sensor_msgs::msg::Imu> imuQueOpt;
  std::deque<sensor_msgs::msg::Imu> imuQueImu;

  gtsam::Pose3 prevPose_;
  gtsam::Vector3 prevVel_;
  gtsam::NavState prevState_;
  gtsam::imuBias::ConstantBias prevBias_;

  gtsam::NavState prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool doneFirstOpt = false;
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2 optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values graphValues;

  const double delta_t = 0;

  int key = 1;

  gtsam::Pose3 imu2Lidar;
  gtsam::Pose3 lidar2Imu;

  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<LioSamParams> params_;

  void resetOptimization();
  void resetParams();
  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
  bool failureDetection(
    const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw);
};
} //  namespace lio_sam
#endif //  LIO_SAM_IMU_PREINTEGRATION_HPP_
