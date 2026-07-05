#ifndef LIO_SAM_TRANSFORM_FUSION_HPP_
#define LIO_SAM_TRANSFORM_FUSION_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lio_sam/utility.hpp"


namespace lio_sam
{
class TransformFusion
{
public:
  TransformFusion(
    rclcpp_lifecycle::LifecycleNode * node_,
    std::shared_ptr<LioSamParams> params);
  virtual ~TransformFusion();

private:
  std::mutex mtx;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

  rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
  rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

  Eigen::Isometry3d lidarOdomAffine;
  Eigen::Isometry3d imuOdomAffineFront;
  Eigen::Isometry3d imuOdomAffineBack;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  tf2::Stamped<tf2::Transform> lidar2Baselink;

  double lidarOdomTime = -1;
  deque<nav_msgs::msg::Odometry> imuOdomQueue;

  rclcpp_lifecycle::LifecycleNode * node_;
  std::shared_ptr<LioSamParams> params_;

  Eigen::Isometry3d odom2affine(nav_msgs::msg::Odometry odom);
  void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
  void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
};
} //  namespace lio_sam
#endif // LIO_SAM_TRANSFORM_FUSION_HPP_
