cmake_minimum_required(VERSION 3.0.2)
project(vofod_mid360 VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_filters
  message_generation
  message_runtime
  mid360_ray_msgs
  mrs_msgs
  nodelet
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

# mrs_msgs exports a helper library containing a process main().  It must not
# leak into the pure core/gtest link or it silently replaces gtest_main.
set(VOFOD_CORE_CATKIN_LIBRARIES)
foreach(library ${catkin_LIBRARIES})
  if(NOT "${library}" MATCHES "MrsMsgs_Main")
    list(APPEND VOFOD_CORE_CATKIN_LIBRARIES "${library}")
  endif()
endforeach()

add_message_files(DIRECTORY msgs FILES
  Detection.msg
  Detections.msg
  MapRevisionEvidence.msg
  MapUpdateDiagnostics.msg
  ProfilingInfo.msg
  Status.msg
)

add_service_files(DIRECTORY srv FILES
  QueryVoxels.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VoFODMid360Core VoFODMid360
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_filters
    message_runtime
    mid360_ray_msgs
    mrs_msgs
    nodelet
    pcl_conversions
    pcl_ros
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(VoFODMid360Core
  src/pc_loader.cpp
  src/ray_update.cpp
  src/strict_baseline_core.cpp
  src/voxel_grid_counted.cpp
  src/voxel_grid_weighted.cpp
  src/voxel_map.cpp
)
add_dependencies(VoFODMid360Core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(VoFODMid360Core ${VOFOD_CORE_CATKIN_LIBRARIES})
target_include_directories(VoFODMid360Core SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)

add_library(VoFODMid360 src/vofod_nodelet.cpp)
add_dependencies(VoFODMid360
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(VoFODMid360 VoFODMid360Core ${catkin_LIBRARIES})
target_include_directories(VoFODMid360 SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(vofod_mid360_core_test
    test/test_main.cpp
    test/ray_update_test.cpp
    test/strict_baseline_core_test.cpp
    test/voxel_grid_test.cpp
  )
  if(TARGET vofod_mid360_core_test)
    add_dependencies(vofod_mid360_core_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(vofod_mid360_core_test
      VoFODMid360Core
      ${VOFOD_CORE_CATKIN_LIBRARIES}
    )
    target_include_directories(vofod_mid360_core_test SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS}
    )
  endif()

  add_rostest(test/b0_soft_update.test)
  add_rostest(test/native_init.test)
  add_rostest_gtest(vofod_mid360_production_config_test
    test/production_config.test
    test/production_config_test.cpp
  )
  if(TARGET vofod_mid360_production_config_test)
    target_link_libraries(vofod_mid360_production_config_test
      ${VOFOD_CORE_CATKIN_LIBRARIES}
    )
    target_include_directories(vofod_mid360_production_config_test
      SYSTEM PRIVATE ${catkin_INCLUDE_DIRS}
    )
  endif()
endif()

install(TARGETS VoFODMid360Core VoFODMid360
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/vofod/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/vofod
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES nodelets.xml LICENSE UPSTREAM.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
