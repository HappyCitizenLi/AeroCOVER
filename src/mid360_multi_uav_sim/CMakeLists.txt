cmake_minimum_required(VERSION 3.0.2)
project(mid360_multi_uav_sim)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/benchmark_scenario.py
  scripts/scenario_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/s1_assets_test.py)
  catkin_add_nosetests(test/s01_assets_test.py)
  catkin_add_nosetests(test/s02_assets_test.py)
  catkin_add_nosetests(test/s03_assets_test.py)
  catkin_add_nosetests(test/s04_assets_test.py)
  catkin_add_nosetests(test/s05_assets_test.py)
  catkin_add_nosetests(test/s06_assets_test.py)
  catkin_add_nosetests(test/benchmark_scenarios_test.py)
  catkin_add_nosetests(test/scenario_manager_s03_test.py)
  catkin_add_nosetests(test/scenario_manager_s02_test.py)
  catkin_add_nosetests(test/scenario_manager_s04_test.py)
  catkin_add_nosetests(test/scenario_manager_s05_test.py)
  catkin_add_nosetests(test/scenario_manager_s06_test.py)
  add_rostest(test/gate_s03.test)
  add_rostest(test/gate_s04.test)
  add_rostest(test/gate_s05.test)
endif()
