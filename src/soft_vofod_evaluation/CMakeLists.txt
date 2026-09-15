cmake_minimum_required(VERSION 3.0.2)
project(soft_vofod_evaluation)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/evaluate_bag.py
  scripts/run_benchmark.py
  scripts/run_four_scene_suite.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md requirements-hota.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/runner_test.py)
  catkin_add_nosetests(test/evaluation_test.py)
endif()
