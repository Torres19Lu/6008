cmake_minimum_required(VERSION 3.0.2)
project(g1_costmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
  g1_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/CostmapTuning.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs
    tf2 tf2_ros tf2_eigen pcl_ros pcl_conversions dynamic_reconfigure g1_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Costmap core: ROS-free, std::thread-free layered-costmap logic (grid + the
# three layers + fusion), linked by both the node and the unit tests.
add_library(${PROJECT_NAME}_core
  src/core/costmap_grid.cpp
  src/core/ground_surface.cpp
  src/core/occupancy_accumulator.cpp
  src/core/local_window.cpp
  src/layers/static_layer.cpp
  src/layers/dynamic_layer.cpp
  src/layers/inflation_layer.cpp
  src/layers/layered_costmap.cpp
)

add_executable(${PROJECT_NAME}_node src/costmap_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# The node uses the generated CostmapTuningConfig header + g1_msgs headers.
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_cost_values test/test_cost_values.cpp)
  target_link_libraries(test_cost_values ${PROJECT_NAME}_core)

  catkin_add_gtest(test_costmap_grid test/test_costmap_grid.cpp)
  target_link_libraries(test_costmap_grid ${PROJECT_NAME}_core)

  catkin_add_gtest(test_ground_classifier test/test_ground_classifier.cpp)
  target_link_libraries(test_ground_classifier ${PROJECT_NAME}_core)

  catkin_add_gtest(test_foot_ground test/test_foot_ground.cpp)
  target_link_libraries(test_foot_ground ${PROJECT_NAME}_core)

  catkin_add_gtest(test_ground_surface test/test_ground_surface.cpp)
  target_link_libraries(test_ground_surface ${PROJECT_NAME}_core)

  catkin_add_gtest(test_occupancy_accumulator test/test_occupancy_accumulator.cpp)
  target_link_libraries(test_occupancy_accumulator ${PROJECT_NAME}_core)

  catkin_add_gtest(test_static_layer test/test_static_layer.cpp)
  target_link_libraries(test_static_layer ${PROJECT_NAME}_core)

  catkin_add_gtest(test_dynamic_layer test/test_dynamic_layer.cpp)
  target_link_libraries(test_dynamic_layer ${PROJECT_NAME}_core)

  catkin_add_gtest(test_inflation_layer test/test_inflation_layer.cpp)
  target_link_libraries(test_inflation_layer ${PROJECT_NAME}_core)

  catkin_add_gtest(test_layered_costmap test/test_layered_costmap.cpp)
  target_link_libraries(test_layered_costmap ${PROJECT_NAME}_core)

  catkin_add_gtest(test_local_window test/test_local_window.cpp)
  target_link_libraries(test_local_window ${PROJECT_NAME}_core)

  # Hardware-free launch + config contract check (needs python3-nose).
  catkin_add_nosetests(test/test_costmap_launch.py)
endif()
