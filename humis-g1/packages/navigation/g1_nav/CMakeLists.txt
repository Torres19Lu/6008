cmake_minimum_required(VERSION 3.0.2)
project(g1_nav)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs std_srvs
  actionlib actionlib_msgs tf2 tf2_ros tf2_geometry_msgs g1_msgs
  dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/NavTuning.cfg)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES g1_nav_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs std_srvs
    actionlib actionlib_msgs tf2 tf2_ros tf2_geometry_msgs g1_msgs
    dynamic_reconfigure)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(g1_nav_core src/core/fsm.cpp src/core/progress_watchdog.cpp src/core/retry.cpp)
add_executable(g1_nav_node src/nav_node.cpp)
target_link_libraries(g1_nav_node g1_nav_core ${catkin_LIBRARIES})
add_dependencies(g1_nav_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_fsm test/test_fsm.cpp)
  target_link_libraries(test_fsm g1_nav_core)
  catkin_add_gtest(test_progress_watchdog test/test_progress_watchdog.cpp)
  target_link_libraries(test_progress_watchdog g1_nav_core)
  catkin_add_gtest(test_retry test/test_retry.cpp)
  target_link_libraries(test_retry g1_nav_core)
  catkin_add_nosetests(test/test_nav_launch.py)
  # End-to-end rostest: g1_nav_node against a single python mock node
  # (mock upstreams + action client + verifier). rostest is a test_depend.
  find_package(rostest REQUIRED)
  add_rostest(test/nav_integration.test)
endif()
