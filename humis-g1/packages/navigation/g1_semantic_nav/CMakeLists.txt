cmake_minimum_required(VERSION 3.0.2)
project(g1_semantic_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  gazebo_msgs
  geometry_msgs
  g1_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
  tf
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib gazebo_msgs geometry_msgs g1_msgs nav_msgs rospy std_msgs std_srvs
    tf tf2_geometry_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/semantic_goal_adapter.py
  scripts/sim_cmd_vel_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_goal_selection.py)
  catkin_add_nosetests(test/test_real_launch_contract.py)
endif()
