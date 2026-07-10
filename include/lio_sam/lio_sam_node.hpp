#ifndef LIO_SAM__LIO_SAM_NODE_HPP_
#define LIO_SAM__LIO_SAM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lio_sam/feature_extraction.hpp"
#include "lio_sam/image_projection.hpp"
#include "lio_sam/imu_preintegration.hpp"
#include "lio_sam/map_optimization.hpp"
#include "lio_sam/transform_fusion.hpp"

namespace lio_sam
{
class LioSamNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LioSamNode(const rclcpp::NodeOptions & options);
  virtual ~LioSamNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/);

private:
  void join_threads();
  void destroy_structs();
  void init_structs();
  void init_parameters();

  std::unique_ptr<ImageProjection> image_projection_;
  std::unique_ptr<FeatureExtraction> feature_extraction_;
  std::unique_ptr<IMUPreintegration> imu_preintegration_;
  std::unique_ptr<MapOptimization> map_optimization_;
  std::unique_ptr<TransformFusion> transform_fusion_;

  std::shared_ptr<LioSamParams> params_;

  std::unique_ptr<std::thread> loopClosureThread_;
  std::unique_ptr<std::thread> visualizeMapThread_;
  std::unique_ptr<std::thread> globalLocalizeThread_;
};
}  // namespace lio_sam
#endif  // LIO_SAM__LIO_SAM_NODE_HPP_
