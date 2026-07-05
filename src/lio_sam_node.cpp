#include "lio_sam/lio_sam_node.hpp"


namespace lio_sam
{
LioSamNode::LioSamNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lio_sam", options)
{
}

LioSamNode::~LioSamNode()
{
  destroy_structs();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LioSamNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "LIO-SAM Configuring...");
  init_structs();
  RCLCPP_INFO(get_logger(), "LIO-SAM Configured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LioSamNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "LIO-SAM Activating...");

  if (params_->global_localization_flag) {
    RCLCPP_INFO(get_logger(), "Global Localization mode: starting globalLocalizeThread");
    globalLocalizeThread_ = std::make_unique<std::thread>(
        &MapOptimization::globalLocalizeThreadFunc, map_optimization_.get());
  } else {
    RCLCPP_INFO(get_logger(), "SLAM mode: starting loopClosure and visualizeGlobalMap threads");
    loopClosureThread_ = std::make_unique<std::thread>(
        &MapOptimization::loopClosureThread, map_optimization_.get());
    visualizeMapThread_ = std::make_unique<std::thread>(
        &MapOptimization::visualizeGlobalMapThread, map_optimization_.get());
  }

  RCLCPP_INFO(get_logger(), "LIO-SAM Activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LioSamNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "LIO-SAM Deactivating...");

  if (globalLocalizeThread_ && globalLocalizeThread_->joinable()) {
    globalLocalizeThread_->join();
    globalLocalizeThread_.reset();
  }
  if (loopClosureThread_ && loopClosureThread_->joinable()) {
    loopClosureThread_->join();
    loopClosureThread_.reset();
  }
  if (visualizeMapThread_ && visualizeMapThread_->joinable()) {
    visualizeMapThread_->join();
    visualizeMapThread_.reset();
  }

  RCLCPP_INFO(get_logger(), "LIO-SAM Deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LioSamNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "LIO-SAM Cleaning up...");
  destroy_structs();
  RCLCPP_INFO(get_logger(), "LIO-SAM Cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LioSamNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "LIO-SAM Shutting down...");
  destroy_structs();
  RCLCPP_INFO(get_logger(), "LIO-SAM Shut down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LioSamNode::destroy_structs()
{
  image_projection_.reset();
  feature_extraction_.reset();
  imu_preintegration_.reset();
  map_optimization_.reset();
  transform_fusion_.reset();
}

void LioSamNode::init_structs()
{
  init_parameters();

  image_projection_ = std::make_unique<ImageProjection>(this, params_);
  RCLCPP_INFO(this->get_logger(), "Configured image projection");
  feature_extraction_ = std::make_unique<FeatureExtraction>(this, params_);
  RCLCPP_INFO(this->get_logger(), "Configured feature extraction");
  imu_preintegration_ = std::make_unique<IMUPreintegration>(this, params_);
  RCLCPP_INFO(this->get_logger(), "Configured IMU preintegration");
  map_optimization_ = std::make_unique<MapOptimization>(this, params_);
  RCLCPP_INFO(this->get_logger(), "Configured map optimization");
  transform_fusion_ = std::make_unique<TransformFusion>(this, params_);
  RCLCPP_INFO(this->get_logger(), "Configured transform fusion");
}

void LioSamNode::init_parameters()
{
  // This function is for future use.
  params_ = std::make_shared<LioSamParams>();

  // Topics
  params_->point_cloud_topic = this->declare_parameter<std::string>("pointCloudTopic", "points");
  params_->imu_topic = this->declare_parameter<std::string>("imuTopic", "imu/data");
  params_->odom_topic = this->declare_parameter<std::string>("odomTopic", "lio_sam/odometry/imu");
  params_->gps_topic = this->declare_parameter<std::string>("gpsTopic", "lio_sam/odometry/gps");

  // Frames
  params_->lidar_frame = this->declare_parameter<std::string>("lidarFrame", "laser_data_frame");
  params_->baselink_frame = this->declare_parameter<std::string>("baselinkFrame", "base_link");
  params_->odometry_frame = this->declare_parameter<std::string>("odometryFrame", "odom");
  params_->map_frame = this->declare_parameter<std::string>("mapFrame", "map");

  // GPS Settings
  params_->use_imu_heading_init = this->declare_parameter<bool>("useImuHeadingInitialization", false);
  params_->use_gps_elevation = this->declare_parameter<bool>("useGpsElevation", false);
  params_->gps_covariance_threshold = this->declare_parameter<float>("gpsCovThreshold", 2.0);
  params_->pose_covariance_threshold = this->declare_parameter<float>("poseCovThreshold", 25.0);

  // Export Settings
  params_->save_pcd = this->declare_parameter<bool>("savePCD", false);
  params_->pointcloud_map_directory_path = this->declare_parameter<std::string>("savePCDDirectory", "/Downloads/LOAM/");

  // Sensor Settings
  params_->sensor = this->declare_parameter<std::string>("sensor", "ouster");
  params_->vertical_scan_num = this->declare_parameter<int>("N_SCAN", 64);
  params_->horizontal_scan_num = this->declare_parameter<int>("Horizon_SCAN", 512);
  params_->downsample_rate = this->declare_parameter<int>("downsampleRate", 1);
  params_->lidar_min_range = this->declare_parameter<float>("lidarMinRange", 5.5);
  params_->lidar_max_range = this->declare_parameter<float>("lidarMaxRange", 1000.0);

  // IMU Settings
  params_->imu_rate = this->declare_parameter<float>("imuRate", 200.0);
  params_->imu_acc_noise = this->declare_parameter<float>("imuAccNoise", 3.9939570888238808e-03);
  params_->imu_gyro_noise = this->declare_parameter<float>("imuGyrNoise", 1.5636343949698187e-03);
  params_->imu_acc_bias_density = this->declare_parameter<float>("imuAccBiasN", 6.4356659353532566e-05);
  params_->imu_gyro_bias_density = this->declare_parameter<float>("imuGyrBiasN", 3.5640318696367613e-05);
  params_->imu_gravity = this->declare_parameter<float>("imuGravity", 9.80511);
  params_->imu_rpy_weight = this->declare_parameter<float>("imuRPYWeight", 0.01);

  // IMU to LiDAR
  params_->extRotV = this->declare_parameter<std::vector<double>>("extrinsicRot", {1.0,  0.0,  0.0,
                                                                           0.0,  1.0,  0.0,
                                                                           0.0,  0.0,  1.0});
  params_->extRPYV = this->declare_parameter<std::vector<double>>("extrinsicRPY", {1.0,  0.0,  0.0,
                                                                           0.0,  1.0,  0.0,
                                                                           0.0,  0.0,  1.0});
  params_->extTransV = this->declare_parameter<std::vector<double>>("extrinsicTrans", {0.0, 0.0, 0.0});
  params_->extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_->extRotV.data(), 3, 3);
  params_->extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_->extRPYV.data(), 3, 3);
  params_->extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_->extTransV.data(), 3, 1);
  params_->extQRPY = Eigen::Quaterniond(params_->extRPY);

  // LOAM feature threshold
  params_->edge_threshold = this->declare_parameter<float>("edgeThreshold", 1.0);
  params_->surf_threshold = this->declare_parameter<float>("surfThreshold", 0.1);
  params_->valid_edge_feature_num = this->declare_parameter<int>("edgeFeatureMinValidNum", 10);
  params_->valid_surf_feature_num = this->declare_parameter<int>("surfFeatureMinValidNum", 100);

  // Voxel filter params
  params_->odometry_surf_leaf_size = this->declare_parameter<float>("odometrySurfLeafSize", 0.4);
  params_->mapping_corner_leaf_size = this->declare_parameter<float>("mappingCornerLeafSize", 0.2);
  params_->mapping_surf_leaf_size = this->declare_parameter<float>("mappingSurfLeafSize", 0.4);

  // Robot motion constraint
  params_->z_tolerance = this->declare_parameter<float>("z_tollerance", 1000.0);
  params_->rotation_tolerance = this->declare_parameter<float>("rotation_tollerance", 1000.0);

  // CPU Params
  params_->number_of_cores = this->declare_parameter<int>("numberOfCores", 4);
  params_->mapping_process_interval = this->declare_parameter<double>("mappingProcessInterval", 0.15);

  // Surrounding Map params
  params_->surrounding_keyframe_distance_threshold = this->declare_parameter<float>("surroundingkeyframeAddingDistThreshold", 1.0f);
  params_->surrounding_keyframe_angle_threshold = this->declare_parameter<float>("surroundingkeyframeAddingAngleThreshold", 0.2f);
  params_->surrounding_keyframe_density = this->declare_parameter<float>("surroundingKeyframeDensity", 2.0f);
  params_->surrounding_keyframe_search_radius = this->declare_parameter<float>("surroundingKeyframeSearchRadius", 50.0f);

  // Loop closure
  params_->loop_closure_enable_flag = this->declare_parameter<bool>("loopClosureEnableFlag", true);
  params_->loop_closure_frequency = this->declare_parameter<float>("loopClosureFrequency", 1.0);
  params_->surrounding_keyframe_size = this->declare_parameter<int>("surroundingKeyframeSize", 50);
  params_->history_keyframe_search_radius = this->declare_parameter<float>("historyKeyframeSearchRadius", 15.0f);
  params_->history_keyframe_time_difference = this->declare_parameter<float>("historyKeyframeSearchTimeDiff", 30.0f);
  params_->history_keyframe_search_num = this->declare_parameter<int>("historyKeyframeSearchNum", 25);
  params_->history_keyframe_fitness_score = this->declare_parameter<double>("historyKeyframeFitnessScore", 0.3);
  params_->scan_context_loop_closure_flag = this->declare_parameter<bool>("scanContextLoopClosureFlag", true);
  params_->scan_context_limit_height = this->declare_parameter<double>("scanContextLimitHeight", 2.0);

  // Global map visualization radius
  params_->global_map_visualization_flag = this->declare_parameter<bool>("visualize_enabled", false);
  params_->global_map_visualization_search_radius = this->declare_parameter<float>("globalMapVisualizationSearchRadius", 1000.0f);
  params_->global_map_visualization_pose_density = this->declare_parameter<float>("globalMapVisualizationPoseDensity", 10.0f);
  params_->global_map_visualization_leaf_size = this->declare_parameter<float>("globalMapVisualizationLeafSize", 1.0f);

  // Global Localization
  params_->global_localization_flag = this->declare_parameter<bool>("globalLocalizationFlag", false);
  params_->load_map_file_dir = this->declare_parameter<std::string>("loadMapFileDir", "");

  // Debug log
  params_->show_log = this->declare_parameter<bool>("show_log", false);

  // Print paramters log
  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), "  Point cloud topic: %s", params_->point_cloud_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  IMU topic: %s", params_->imu_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Sensor type: %s", params_->sensor.c_str());
}
}  // namespace lio_sam

#include "rclcpp_components/register_node_macro.hpp"

// Register this class as a component
RCLCPP_COMPONENTS_REGISTER_NODE(lio_sam::LioSamNode)
