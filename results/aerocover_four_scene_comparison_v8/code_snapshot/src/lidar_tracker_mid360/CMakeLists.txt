cmake_minimum_required(VERSION 3.1.2)
project(lidar_tracker_mid360 VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  mrs_lib
  mrs_msgs
  nav_msgs
  nodelet
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  vofod_mid360
)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/covmat.cfg)

add_message_files(DIRECTORY msg FILES
  Track.msg
  TrackerFrameComplete.msg
  Tracks.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LidarTrackerMid360Core LidarTrackerMid360
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    mrs_lib
    mrs_msgs
    nav_msgs
    nodelet
    pcl_conversions
    pcl_ros
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    vofod_mid360
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(LidarTrackerMid360Core src/tracker_core.cpp)
target_include_directories(LidarTrackerMid360Core SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(LidarTrackerMid360 src/lidar_tracker.cpp)
add_dependencies(LidarTrackerMid360
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(LidarTrackerMid360
  LidarTrackerMid360Core
  ${catkin_LIBRARIES}
)
target_include_directories(LidarTrackerMid360 SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(lidar_tracker_mid360_core_test
    test/test_main.cpp
    test/tracker_core_test.cpp
  )
  if(TARGET lidar_tracker_mid360_core_test)
    target_link_libraries(lidar_tracker_mid360_core_test LidarTrackerMid360Core)
    target_include_directories(lidar_tracker_mid360_core_test SYSTEM PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
  add_rostest(test/synthetic_tracker.test)
  add_rostest(test/frame_completion_contract.test)
endif()

install(TARGETS LidarTrackerMid360Core LidarTrackerMid360
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugins.xml LICENSE README.md UPSTREAM.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
