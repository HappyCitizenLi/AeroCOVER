cmake_minimum_required(VERSION 3.0.2)
project(tclv_evaluation)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  mid360_ray_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
)

add_message_files(FILES
  VisibilityGroundTruth.msg
  VisibilityGroundTruthArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tclv_evaluation_geometry
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    mid360_ray_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(tclv_evaluation_geometry src/geometry.cpp)
add_dependencies(tclv_evaluation_geometry
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tclv_evaluation_geometry ${catkin_LIBRARIES})

add_executable(visibility_ground_truth_evaluator
  src/visibility_ground_truth_evaluator.cpp
)
add_dependencies(visibility_ground_truth_evaluator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(visibility_ground_truth_evaluator
  tclv_evaluation_geometry
  ${catkin_LIBRARIES}
)

install(TARGETS
    tclv_evaluation_geometry
    visibility_ground_truth_evaluator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES launch/b0_canonical.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(tclv_evaluation_geometry_test test/geometry_test.cpp)
  if(TARGET tclv_evaluation_geometry_test)
    target_link_libraries(tclv_evaluation_geometry_test
      tclv_evaluation_geometry
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(tclv_evaluation_truth_time_test
    test/truth_time_test.cpp
  )
  if(TARGET tclv_evaluation_truth_time_test)
    target_link_libraries(tclv_evaluation_truth_time_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(tclv_evaluation_truth_source_contract_test
    test/truth_source_contract_test.cpp
  )
  if(TARGET tclv_evaluation_truth_source_contract_test)
    target_link_libraries(tclv_evaluation_truth_source_contract_test
      ${catkin_LIBRARIES}
    )
  endif()
endif()
