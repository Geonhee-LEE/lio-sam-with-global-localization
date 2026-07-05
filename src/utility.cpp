#include "lio_sam/utility.hpp"

namespace lio_sam {
rmw_qos_profile_t qos_profile{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                              1,
                              RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                              RMW_QOS_POLICY_DURABILITY_VOLATILE,
                              RMW_QOS_DEADLINE_DEFAULT,
                              RMW_QOS_LIFESPAN_DEFAULT,
                              RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                              RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                              false};

rclcpp::QoS qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

rmw_qos_profile_t qos_profile_imu{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                  2000,
                                  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                  RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                  RMW_QOS_DEADLINE_DEFAULT,
                                  RMW_QOS_LIFESPAN_DEFAULT,
                                  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                  false};

rclcpp::QoS qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile_imu.history, qos_profile_imu.depth),
    qos_profile_imu);

rmw_qos_profile_t qos_profile_lidar{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    5,
                                    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                    RMW_QOS_DEADLINE_DEFAULT,
                                    RMW_QOS_LIFESPAN_DEFAULT,
                                    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                    false};

rclcpp::QoS qos_lidar =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_lidar.history,
                                          qos_profile_lidar.depth),
                qos_profile_lidar);

sensor_msgs::msg::Imu
imuConverter(const sensor_msgs::msg::Imu &imu_in,
             const Eigen::Matrix3d &lidar_to_imu_rotation,
             const Eigen::Quaterniond &imu_to_lidar_quaternion) {
  sensor_msgs::msg::Imu imu_out = imu_in;
  // rotate acceleration
  Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                      imu_in.linear_acceleration.y,
                      imu_in.linear_acceleration.z);
  acc = lidar_to_imu_rotation * acc;
  imu_out.linear_acceleration.x = acc.x();
  imu_out.linear_acceleration.y = acc.y();
  imu_out.linear_acceleration.z = acc.z();
  // rotate gyroscope
  Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                      imu_in.angular_velocity.z);
  gyr = lidar_to_imu_rotation * gyr;
  imu_out.angular_velocity.x = gyr.x();
  imu_out.angular_velocity.y = gyr.y();
  imu_out.angular_velocity.z = gyr.z();
  // rotate roll pitch yaw
  Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
                            imu_in.orientation.y, imu_in.orientation.z);
  Eigen::Quaterniond q_final = q_from * imu_to_lidar_quaternion;
  imu_out.orientation.x = q_final.x();
  imu_out.orientation.y = q_final.y();
  imu_out.orientation.z = q_final.z();
  imu_out.orientation.w = q_final.w();

  // if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() +
  // q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
  // {
  // 		RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a
  // 9-axis IMU!"); 		rclcpp::shutdown();
  // }

  return imu_out;
}

sensor_msgs::msg::PointCloud2 publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
    pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud, rclcpp::Time thisStamp,
    std::string thisFrame) {
  sensor_msgs::msg::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub->get_subscription_count() != 0)
    thisPub->publish(tempCloud);
  return tempCloud;
}

float pointDistance(pcl::PointXYZI p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(pcl::PointXYZI p1, pcl::PointXYZI p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

void saveSCD(
  std::string fileName, Eigen::MatrixXd matrix,
  std::string delimiter)
{
  // delimiter: ", " or " " etc.

  int precision = 3; // or Eigen::FullPrecision, but SCD does not require such
                     // accruate precisions so 3 is enough.
  const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols,
                                          delimiter, "\n");

  std::ofstream file(fileName);
  if (file.is_open()) {
    file << matrix.format(the_format);
    file.close();
  }
}

std::string padZeros(int val, int num_digits) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}
} //  namespace lio_sam