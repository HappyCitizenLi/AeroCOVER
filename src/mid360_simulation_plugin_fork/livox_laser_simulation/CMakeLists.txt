cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  roslib
  tf
  sensor_msgs
  std_msgs
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
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS diagnostic_msgs message_runtime mid360_ray_msgs roscpp roslib sensor_msgs std_msgs tf
)

add_library(livox_laser_simulation SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
  src/per_ray_geometry.cpp
)
add_dependencies(livox_laser_simulation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)

target_link_libraries(livox_laser_simulation libprotobuf.so.9)

catkin_install_python(
  PROGRAMS
    scripts/b_t4_per_ray_wall_validator.py
    scripts/ray_bundle_validator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(per_ray_geometry_test
    test/per_ray_geometry_test.cpp
    src/per_ray_geometry.cpp
  )
  add_rostest(test/b_t4_per_ray_stationary_wall.test)
  add_rostest(test/b_t4_per_ray_yaw_wall.test)
  add_rostest(test/ray_bundle_empty.test)
  add_rostest(test/ray_bundle_wall.test)
endif()
