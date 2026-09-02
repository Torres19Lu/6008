cmake_minimum_required(VERSION 3.0.2)
project(g1_global_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

# Live-tunable knobs (added with the node, Task 6). Named PlannerTuning so the
# generated PlannerTuningConfig does not clash with the core PlannerConfig struct.
generate_dynamic_reconfigure_options(
  cfg/PlannerTuning.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs nav_msgs
    tf2 tf2_ros tf2_geometry_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Core library: ROS-free A* logic (grid + cost model + search + smoother + facade).
# Linked by both the node (Task 6) and the unit tests below.
add_library(${PROJECT_NAME}_core
  src/core/planner_grid.cpp
  src/core/cost_model.cpp
  src/planner/astar.cpp
  src/planner/path_smoother.cpp
  src/planner/global_planner.cpp
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
# ROS node: thin I/O wrapper that drives the core on a timer.
add_executable(${PROJECT_NAME}_node src/planner_node.cpp)
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
  catkin_add_gtest(test_planner_grid test/test_planner_grid.cpp)
  target_link_libraries(test_planner_grid ${PROJECT_NAME}_core)

  catkin_add_gtest(test_cost_model test/test_cost_model.cpp)
  target_link_libraries(test_cost_model ${PROJECT_NAME}_core)

  catkin_add_gtest(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar ${PROJECT_NAME}_core)

  catkin_add_gtest(test_path_smoother test/test_path_smoother.cpp)
  target_link_libraries(test_path_smoother ${PROJECT_NAME}_core)

  catkin_add_gtest(test_global_planner test/test_global_planner.cpp)
  target_link_libraries(test_global_planner ${PROJECT_NAME}_core)

  # Hardware-free launch + config contract check (needs python3-nose).
  catkin_add_nosetests(test/test_global_planner_launch.py)
endif()
