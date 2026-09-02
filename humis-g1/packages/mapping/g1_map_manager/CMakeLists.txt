cmake_minimum_required(VERSION 3.0.2)
project(g1_map_manager)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs geometry_msgs nav_msgs std_srvs
  actionlib actionlib_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs
  visualization_msgs sensor_msgs pcl_conversions g1_msgs)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES g1_map_manager_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs std_srvs
    actionlib actionlib_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs
    visualization_msgs sensor_msgs pcl_conversions g1_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

# ROS-free core (Eigen + yaml-cpp only; no ROS headers). Sources are appended as
# each core unit lands (version_store, topology_graph, overlay, nav_point_store,
# mode_machine).
add_library(g1_map_manager_core
  src/core/pose_math.cpp
  src/core/map_lineage.cpp
  src/core/map_store.cpp
  src/core/topology_graph.cpp
  src/core/overlay.cpp
  src/core/nav_point_store.cpp
  src/core/mode_machine.cpp
  src/core/gateway_editor.cpp
  src/core/gateway_capture.cpp
  src/core/map_editor.cpp)
target_link_libraries(g1_map_manager_core ${YAML_CPP_LIBRARIES})

add_executable(g1_map_manager_node src/map_manager_node.cpp)
target_link_libraries(g1_map_manager_node g1_map_manager_core ${catkin_LIBRARIES})
add_dependencies(g1_map_manager_node ${catkin_EXPORTED_TARGETS})

add_executable(g1_map_edit_node src/map_edit_node.cpp)
target_link_libraries(g1_map_edit_node g1_map_manager_core ${catkin_LIBRARIES})
add_dependencies(g1_map_edit_node ${catkin_EXPORTED_TARGETS})

add_executable(g1_keyframe_overlay_node src/keyframe_overlay_node.cpp)
target_link_libraries(g1_keyframe_overlay_node g1_map_manager_core ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
add_dependencies(g1_keyframe_overlay_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/map_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_map_types test/test_map_types.cpp)
  target_link_libraries(test_map_types g1_map_manager_core)
  catkin_add_gtest(test_map_lineage test/test_map_lineage.cpp)
  target_link_libraries(test_map_lineage g1_map_manager_core)
  catkin_add_gtest(test_map_store test/test_map_store.cpp)
  target_link_libraries(test_map_store g1_map_manager_core)
  catkin_add_gtest(test_topology_graph test/test_topology_graph.cpp)
  target_link_libraries(test_topology_graph g1_map_manager_core)
  catkin_add_gtest(test_overlay test/test_overlay.cpp)
  target_link_libraries(test_overlay g1_map_manager_core)
  catkin_add_gtest(test_nav_point_store test/test_nav_point_store.cpp)
  target_link_libraries(test_nav_point_store g1_map_manager_core)
  catkin_add_gtest(test_mode_machine test/test_mode_machine.cpp)
  target_link_libraries(test_mode_machine g1_map_manager_core)
  catkin_add_gtest(test_gateway_edit test/test_gateway_edit.cpp)
  target_link_libraries(test_gateway_edit g1_map_manager_core)
  catkin_add_gtest(test_gateway_capture test/test_gateway_capture.cpp)
  target_link_libraries(test_gateway_capture g1_map_manager_core)
  catkin_add_gtest(test_map_edit test/test_map_edit.cpp)
  target_link_libraries(test_map_edit g1_map_manager_core)
  catkin_add_gtest(test_keyframe_overlay test/test_keyframe_overlay.cpp)
  target_link_libraries(test_keyframe_overlay g1_map_manager_core ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

  # Hardware-free launch + config contract checks (needs python3-nose).
  catkin_add_nosetests(test/test_map_manager_launch.py)

  # Integration rostest: map_manager_node against a single python mock backend.
  find_package(rostest REQUIRED)
  add_rostest(test/map_manager_integration.test)
endif()
