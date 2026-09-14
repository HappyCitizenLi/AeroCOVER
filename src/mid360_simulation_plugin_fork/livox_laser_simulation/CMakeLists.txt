cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  diagnostic_msgs
  geometry_msgs
  roscpp
  roslib
  tf
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_msgs
  tf2
  message_generation
  mid360_ray_msgs
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
  #   Message2.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
)
#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mrs_livox_mid360_adapter
        CATKIN_DEPENDS diagnostic_msgs geometry_msgs message_runtime mid360_ray_msgs roscpp roslib sensor_msgs std_msgs tf tf2 tf2_ros
)

catkin_add_env_hooks(50.mid360_gazebo_plugin
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

# The MRS X500 Jinja model loads this exact library name.  Build the same
# Mid360 ray/identity adapter under that name so the rest of the vehicle stays
# entirely on the stock MRS simulation chain.
add_library(mrs_livox_mid360_adapter SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
  src/per_ray_geometry.cpp
)
set_target_properties(mrs_livox_mid360_adapter PROPERTIES
  OUTPUT_NAME MrsGazeboCommonResources_LivoxLaserSimulationPlugin
)
add_dependencies(mrs_livox_mid360_adapter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mrs_livox_mid360_adapter
  ${catkin_LIBRARIES} RayPlugin libprotobuf.so.9
)

# The existing Gazebo env hook puts this workspace library before /opt/ros.
add_library(mrs_ouster_gpu_adapter SHARED src/mrs_3dlidar_plugin.cpp)
target_compile_definitions(mrs_ouster_gpu_adapter PRIVATE GAZEBO_GPU_RAY=1)
set_target_properties(mrs_ouster_gpu_adapter PROPERTIES
  OUTPUT_NAME MrsGazeboCommonResources_3DLidarGpuPlugin)
target_link_libraries(mrs_ouster_gpu_adapter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} GpuRayPlugin)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(scan_stamp_gate_test test/scan_stamp_gate_test.cpp)
  catkin_add_gtest(rolling_scan_schedule_test
    test/rolling_scan_schedule_test.cpp
  )
  if(TARGET rolling_scan_schedule_test)
    add_dependencies(rolling_scan_schedule_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(rolling_scan_schedule_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS mrs_livox_mid360_adapter mrs_ouster_gpu_adapter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
