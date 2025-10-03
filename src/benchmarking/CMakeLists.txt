cmake_minimum_required(VERSION 3.8)
project(benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(uncertain_pointcloud_msgs REQUIRED)   
find_package(gauss_splat REQUIRED)
find_package(fcl REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(uncertain_octomap REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})



set(HEADER_FILES
  include/${PROJECT_NAME}/benchmarking.hpp
)

set(SOURCE_FILES
  
  src/benchmarking.cpp
  src/benchmarking_node.cpp

)


include_directories(include)


link_directories(
  include
)

add_executable(benchmarking_node ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(benchmarking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(benchmarking_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS benchmarking_node
  DESTINATION lib/${PROJECT_NAME})



ament_target_dependencies(benchmarking_node
  rclcpp
  OpenMP
  uncertain_pointcloud_msgs
  pcl_ros
  gauss_splat
  nav_msgs
  std_srvs
  ompl
  fcl
  visualization_msgs
  octomap_msgs
  uncertain_octomap
)


add_executable(unseen_active_perception_node include/${PROJECT_NAME}/unseen_active_perception.hpp src/unseen_active_perception.cpp src/unseen_active_perception_node.cpp)
target_include_directories(unseen_active_perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(unseen_active_perception_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS unseen_active_perception_node
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(unseen_active_perception_node
  rclcpp
  OpenMP
  uncertain_pointcloud_msgs
  pcl_ros
  gauss_splat
  nav_msgs
  std_srvs
  ompl
  fcl
  visualization_msgs
  octomap_msgs
  uncertain_octomap
)



install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    resource/
  DESTINATION share/${PROJECT_NAME}/resource 
)
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()





ament_package()

