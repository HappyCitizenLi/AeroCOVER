cmake_minimum_required(VERSION 3.0.2)
project(aerocover_mid360 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  aerocover_st_background
  diagnostic_msgs
  geometry_msgs
  lidar_tracker_mid360
  message_filters
  message_generation
  message_runtime
  mid360_ray_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

add_message_files(FILES
  AeroCoverDiagnostics.msg
  AeroCoverTrack.msg
  BirthEvent.msg
  CandidateEvidence.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES AeroCoverCore
  CATKIN_DEPENDS
    aerocover_st_background diagnostic_msgs geometry_msgs lidar_tracker_mid360 message_filters message_runtime
    mid360_ray_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
    visualization_msgs
)

# lidar_tracker_mid360 is used only for its Tracks message ABI. Its implementation
# libraries (and their VoFOD dependencies) must not enter the AeroCOVER binary.
set(AEROCOVER_CATKIN_LIBRARIES)
foreach(library ${catkin_LIBRARIES})
  if(NOT "${library}" MATCHES
      "MrsMsgs_Main|LidarTrackerMid360|VoFODMid360")
    list(APPEND AEROCOVER_CATKIN_LIBRARIES "${library}")
  endif()
endforeach()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(AeroCoverCore
  src/ca_tracker.cpp
  src/ray_geometry.cpp
  src/aerocover_core.cpp
)
add_dependencies(AeroCoverCore
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(AeroCoverCore ${AEROCOVER_CATKIN_LIBRARIES})
target_include_directories(AeroCoverCore SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(aerocover_node src/aerocover_node.cpp)
add_dependencies(aerocover_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(aerocover_node AeroCoverCore ${AEROCOVER_CATKIN_LIBRARIES})
target_include_directories(aerocover_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(aerocover_core_test
    test/test_main.cpp
    test/aerocover_core_test.cpp
  )
  if(TARGET aerocover_core_test)
    add_dependencies(aerocover_core_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(aerocover_core_test
      AeroCoverCore ${AEROCOVER_CATKIN_LIBRARIES})
    target_include_directories(aerocover_core_test SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS})
  endif()
endif()

install(TARGETS AeroCoverCore aerocover_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/aerocover_mid360/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/aerocover_mid360)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
