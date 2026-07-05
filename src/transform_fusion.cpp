#include "lio_sam/transform_fusion.hpp"


namespace lio_sam
{
TransformFusion::TransformFusion(
  rclcpp_lifecycle::LifecycleNode * node_,
  std::shared_ptr<LioSamParams> params)
: node_(node_), params_(params)
{
  tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  callbackGroupImuOdometry = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackGroupLaserOdometry = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  auto imuOdomOpt = rclcpp::SubscriptionOptions();
  imuOdomOpt.callback_group = callbackGroupImuOdometry;
  auto laserOdomOpt = rclcpp::SubscriptionOptions();
  laserOdomOpt.callback_group = callbackGroupLaserOdometry;

  subLaserOdometry = node_->create_subscription<nav_msgs::msg::Odometry>(
    "lio_sam/mapping/odometry", qos,
    std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
    laserOdomOpt);
  subImuOdometry = node_->create_subscription<nav_msgs::msg::Odometry>(
    params_->odom_topic+"_incremental", qos_imu,
    std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
    imuOdomOpt);

  pubImuOdometry = node_->create_publisher<nav_msgs::msg::Odometry>(params_->odom_topic, qos_imu);
  pubImuPath = node_->create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);

  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

TransformFusion::~TransformFusion()
{}

Eigen::Isometry3d TransformFusion::odom2affine(nav_msgs::msg::Odometry odom)
{
  tf2::Transform t;
  tf2::fromMsg(odom.pose.pose, t);
  return tf2::transformToEigen(tf2::toMsg(t));
}

void TransformFusion::lidarOdometryHandler(
  const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  std::lock_guard<std::mutex> lock(mtx);

  lidarOdomAffine = odom2affine(*odomMsg);

  lidarOdomTime = stamp2Sec(odomMsg->header.stamp);
}

void TransformFusion::imuOdometryHandler(
  const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  std::lock_guard<std::mutex> lock(mtx);

  imuOdomQueue.push_back(*odomMsg);

  // get latest odometry (at current IMU stamp)
  if (lidarOdomTime == -1)
      return;
  while (!imuOdomQueue.empty())
  {
    if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
      imuOdomQueue.pop_front();
    else
      break;
  }
  Eigen::Isometry3d imuOdomAffineFront = odom2affine(imuOdomQueue.front());
  Eigen::Isometry3d imuOdomAffineBack = odom2affine(imuOdomQueue.back());
  Eigen::Isometry3d imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
  Eigen::Isometry3d imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
  auto t = tf2::eigenToTransform(imuOdomAffineLast);
  tf2::Stamped<tf2::Transform> tCur;
  tf2::convert(t, tCur);

  // publish latest odometry
  nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
  laserOdometry.pose.pose.position.x = t.transform.translation.x;
  laserOdometry.pose.pose.position.y = t.transform.translation.y;
  laserOdometry.pose.pose.position.z = t.transform.translation.z;
  laserOdometry.pose.pose.orientation = t.transform.rotation;
  pubImuOdometry->publish(laserOdometry);

  // publish tf
  if(params_->lidar_frame != params_->baselink_frame)
  {
    try
    {
      tf2::fromMsg(tfBuffer->lookupTransform(
        params_->lidar_frame, params_->baselink_frame, rclcpp::Time(0)), lidar2Baselink);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
    tf2::Stamped<tf2::Transform> tb(
      tCur * lidar2Baselink, tf2_ros::fromMsg(odomMsg->header.stamp), params_->odometry_frame);
    tCur = tb;
  }
  geometry_msgs::msg::TransformStamped ts;
  tf2::convert(tCur, ts);
  ts.header.stamp = odomMsg->header.stamp;
  ts.header.frame_id = params_->odometry_frame;
  ts.child_frame_id = params_->baselink_frame;
  tfBroadcaster->sendTransform(ts);

  // publish IMU path
  static nav_msgs::msg::Path imuPath;
  static double last_path_time = -1;
  double imuTime = stamp2Sec(imuOdomQueue.back().header.stamp);
  if (imuTime - last_path_time > 0.1)
  {
    last_path_time = imuTime;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
    pose_stamped.header.frame_id = params_->odometry_frame;
    pose_stamped.pose = laserOdometry.pose.pose;
    imuPath.poses.push_back(pose_stamped);
    while(!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
      imuPath.poses.erase(imuPath.poses.begin());
    if (pubImuPath->get_subscription_count() != 0)
    {
      imuPath.header.stamp = imuOdomQueue.back().header.stamp;
      imuPath.header.frame_id = params_->odometry_frame;
      pubImuPath->publish(imuPath);
    }
  }
}
} // namespace lio_sam

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.use_intra_process_comms(true);
//   rclcpp::executors::MultiThreadedExecutor exec;

//   auto TF = std::make_shared<lio_sam::TransformFusion>(options);
//   exec.add_node(TF);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Transform Fusion Started.\033[0m");
//   exec.spin();

//   rclcpp::shutdown();
//   return 0;
// }
