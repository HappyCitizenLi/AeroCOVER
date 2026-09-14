cmake_minimum_required(VERSION 3.0.2)
project(mid360_ray_preprocessor)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_filters
  mid360_ray_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mid360_tf_stamp_cache
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_filters
    mid360_ray_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mid360_tf_stamp_cache
  src/tf_stamp_cache.cpp
)
target_compile_options(mid360_tf_stamp_cache PRIVATE -Werror)

add_executable(mid360_ray_preprocessor_node
  src/mid360_ray_preprocessor_node.cpp
)
add_dependencies(mid360_ray_preprocessor_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mid360_ray_preprocessor_node
  mid360_tf_stamp_cache
  ${catkin_LIBRARIES}
)

add_executable(ouster_snapshot_adapter_node
  src/ouster_snapshot_adapter_node.cpp
)
add_dependencies(ouster_snapshot_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ouster_snapshot_adapter_node ${catkin_LIBRARIES})

install(TARGETS mid360_tf_stamp_cache mid360_ray_preprocessor_node
  ouster_snapshot_adapter_node
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
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(mid360_tf_stamp_cache_test test/tf_stamp_cache_test.cpp)
  if(TARGET mid360_tf_stamp_cache_test)
    target_link_libraries(mid360_tf_stamp_cache_test mid360_tf_stamp_cache)
    target_compile_options(mid360_tf_stamp_cache_test PRIVATE -Werror)
  endif()
  catkin_add_gtest(mid360_source_mode_test test/source_mode_test.cpp)
  if(TARGET mid360_source_mode_test)
    target_compile_options(mid360_source_mode_test PRIVATE -Werror)
  endif()
  add_rostest(test/preprocessor_snapshot.test)
  add_rostest(test/preprocessor_rolling_scene.test)
  add_rostest(test/preprocessor_source_mode.test)
  add_rostest(test/ouster_mask_burst.test)
endif()
