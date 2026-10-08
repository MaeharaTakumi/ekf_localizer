cmake_minimum_required(VERSION 3.8)
project(ekf_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ROS に依存しない EKF 本体（ノードとテストで共用）
add_library(ekf_core STATIC
  src/ekf_core.cpp
  src/observation_model.cpp
  src/ndt_model.cpp
  src/odom_model.cpp
  src/lagged_ekf.cpp
)
target_include_directories(ekf_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ekf_localizer_node
  src/ekf_params.cpp
  src/ekf_localizer_node.cpp
  src/ekf_localizer_main.cpp
)
target_link_libraries(ekf_localizer_node ekf_core)
ament_target_dependencies(ekf_localizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

install(TARGETS ekf_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # EKF の単体テスト（ROS 非依存）。colcon test --packages-select ekf_localizer
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf_models test/test_ekf_models.cpp)
  target_link_libraries(test_ekf_models ekf_core)
  ament_add_gtest(test_lagged_ekf test/test_lagged_ekf.cpp)
  target_link_libraries(test_lagged_ekf ekf_core)
endif()

ament_package()
