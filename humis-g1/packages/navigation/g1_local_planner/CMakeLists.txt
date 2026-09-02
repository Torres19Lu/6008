cmake_minimum_required(VERSION 3.0.2)
project(g1_local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# Live-tunable knobs. Named LocalPlannerTuning to avoid clashing
# with the core MpcConfig struct.
generate_dynamic_reconfigure_options(cfg/LocalPlannerTuning.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs nav_msgs std_srvs
    tf2 tf2_ros tf2_geometry_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Core library: ROS-free MPC logic (costmap view; extended in Tasks 2-5 with
# kinematic model, path reference, QP assembly, and the LocalPlanner facade).
# Linked by both the node and the unit tests below.
add_library(${PROJECT_NAME}_core
  src/core/costmap_view.cpp
  src/core/kinematic_model.cpp
  src/core/robot_footprint.cpp
  src/core/footprint_safety.cpp    # footprint collision check
  src/planner/path_reference.cpp
  src/planner/mpc.cpp
  src/planner/local_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  OsqpEigen::OsqpEigen
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# ---------------------------------------------------------------------------
# ROS node: thin I/O wrapper that drives the core on a 20 Hz timer.
add_executable(${PROJECT_NAME}_node src/local_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# ---------------------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_robot_footprint test/test_robot_footprint.cpp)
  target_link_libraries(test_robot_footprint ${PROJECT_NAME}_core ${catkin_LIBRARIES})

  catkin_add_gtest(test_costmap_view test/test_costmap_view.cpp)
  target_link_libraries(test_costmap_view ${PROJECT_NAME}_core)

  catkin_add_gtest(test_kinematic_model test/test_kinematic_model.cpp)
  target_link_libraries(test_kinematic_model ${PROJECT_NAME}_core)

  catkin_add_gtest(test_path_reference test/test_path_reference.cpp)
  target_link_libraries(test_path_reference ${PROJECT_NAME}_core)

  catkin_add_gtest(test_mpc test/test_mpc.cpp)
  target_link_libraries(test_mpc ${PROJECT_NAME}_core)

  catkin_add_gtest(test_local_planner test/test_local_planner.cpp)
  target_link_libraries(test_local_planner ${PROJECT_NAME}_core)

  catkin_add_gtest(test_footprint_safety test/test_footprint_safety.cpp)
  target_link_libraries(test_footprint_safety ${PROJECT_NAME}_core ${catkin_LIBRARIES})

  catkin_add_nosetests(test/test_local_planner_launch.py)
endif()
