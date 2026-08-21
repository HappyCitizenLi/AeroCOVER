cmake_minimum_required(VERSION 3.0.2)
project(mid360_ray_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Ray.msg
  RayBundle.msg
  CheckedRay.msg
  CheckedRayBundle.msg
  ScanIdentity.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)
