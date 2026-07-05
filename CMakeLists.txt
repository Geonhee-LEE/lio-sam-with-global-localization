################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.12)
project(lio_sam_with_global_localization)

if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO "$ENV{ROS_DISTRO}")
    message("Detected ROS_DISTRO: ${ROS_DISTRO}")
else()
    message("ROS_DISTRO environment variable is not set. Please set it in your environment.")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-ignored-qualifiers -Wno-reorder -Wall -Wextra -Wno-unused-parameter)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

################################################################################
# Build
################################################################################
if($ENV{BUILD_TYPE} MATCHES "Release")
  set(CMAKE_BUILD_TYPE Release)
  message("Build type is Release")
else()
  set(CMAKE_BUILD_TYPE Debug)
  message("Build type is Debug")
endif()

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)

set(LIB_NAME ${PROJECT_NAME}_lib)

add_library(${LIB_NAME} SHARED
  src/feature_extraction.cpp
  src/image_projection.cpp
  src/imu_preintegration.cpp
  src/map_optimization.cpp
  src/transform_fusion.cpp
  src/utility.cpp
  src/lio_sam_node.cpp
)

ament_target_dependencies(${LIB_NAME}
  geometry_msgs
  GTSAM
  nav_msgs
  OpenCV
  pcl_conversions
  pcl_msgs
  PCL
  rclcpp
  rclcpp_components
  rclpy
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  rclcpp_lifecycle
  cv_bridge
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(${LIB_NAME} gtsam OpenMP::OpenMP_CXX ${PCL_LIBRARIES} "${cpp_typesupport_target}")
endif()

set(EXECUTABLE_NAME lio_sam_node)
add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${LIB_NAME})

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck()
  ament_cpplint()
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_uncrustify()
  ament_xmllint()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclpy)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(pcl_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_sensor_msgs)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(OpenCV)
ament_export_dependencies(PCL)
ament_export_dependencies(GTSAM)
ament_export_dependencies(Eigen3)
ament_export_dependencies(OpenMP)
ament_export_dependencies(rclcpp_lifecycle)
ament_package()

