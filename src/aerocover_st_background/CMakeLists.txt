cmake_minimum_required(VERSION 3.0.2)
project(aerocover_st_background VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES AeroCoverSTBackground
)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(AeroCoverSTBackground src/background.cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(aerocover_st_background_test
    test/test_main.cpp
    test/background_test.cpp)
  if(TARGET aerocover_st_background_test)
    target_link_libraries(aerocover_st_background_test AeroCoverSTBackground)
  endif()
endif()

install(TARGETS AeroCoverSTBackground
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/aerocover_st_background/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/aerocover_st_background)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
