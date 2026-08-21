cmake_minimum_required(VERSION 3.0.2)
project(soft_vofod_mid360 VERSION 0.1.0)

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
  roscpp
  sensor_msgs
  std_msgs
  vofod_mid360
)
find_package(Eigen3 REQUIRED)

add_message_files(FILES
  FreeSpaceViolationEvent.msg
  FreeSpaceViolationEvents.msg
  OpportunityDebug.msg
  SoftTrack.msg
  SoftTracks.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SoftVofodCore
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_filters
    message_runtime
    mid360_ray_msgs
    roscpp
    sensor_msgs
    std_msgs
    vofod_mid360
)

# vofod_mid360 exports mrs_msgs transitively.  Its process-main helper must not
# replace this package's executable/gtest main.
set(SOFT_VOFOD_CATKIN_LIBRARIES)
foreach(library ${catkin_LIBRARIES})
  if(NOT "${library}" MATCHES "MrsMsgs_Main")
    list(APPEND SOFT_VOFOD_CATKIN_LIBRARIES "${library}")
  endif()
endforeach()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(SoftVofodCore src/core.cpp)
add_dependencies(SoftVofodCore
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(SoftVofodCore ${SOFT_VOFOD_CATKIN_LIBRARIES})
target_include_directories(SoftVofodCore SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(soft_vofod_node src/soft_vofod_node.cpp)
add_dependencies(soft_vofod_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(soft_vofod_node
  SoftVofodCore
  ${SOFT_VOFOD_CATKIN_LIBRARIES}
)
target_include_directories(soft_vofod_node SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(soft_vofod_core_test
    test/test_main.cpp
    test/core_test.cpp
  )
  if(TARGET soft_vofod_core_test)
    add_dependencies(soft_vofod_core_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(soft_vofod_core_test
      SoftVofodCore
      ${SOFT_VOFOD_CATKIN_LIBRARIES}
    )
    target_include_directories(soft_vofod_core_test SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS}
    )
  endif()
  add_rostest(test/pipeline.test)
endif()

install(TARGETS SoftVofodCore soft_vofod_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/soft_vofod_mid360/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/soft_vofod_mid360
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
