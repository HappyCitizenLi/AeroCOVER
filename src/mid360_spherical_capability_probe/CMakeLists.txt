cmake_minimum_required(VERSION 3.0.2)
project(mid360_spherical_capability_probe)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  mid360_ray_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/mid360_spherical_capability_probe
  scripts/seal_software_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/probe_core_test.py)
endif()
