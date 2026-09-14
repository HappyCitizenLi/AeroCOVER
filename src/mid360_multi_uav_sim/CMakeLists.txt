cmake_minimum_required(VERSION 3.0.2)
project(mid360_multi_uav_sim)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/benchmark_scenario.py
  scripts/rangefinder_follower.py
  scripts/view_scenario.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/benchmark_scenarios_test.py)
endif()
