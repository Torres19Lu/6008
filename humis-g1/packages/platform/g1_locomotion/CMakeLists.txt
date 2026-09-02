cmake_minimum_required(VERSION 3.0.2)
project(g1_locomotion)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_srvs
  g1_msgs
)
find_package(unitree_sdk2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_srvs g1_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Pure (SDK-free, ROS-free) safety/translation core.
add_library(g1_loco_core
  src/loco_safety.cpp
  src/joint_map.cpp
)
add_dependencies(g1_loco_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(g1_loco_core ${catkin_LIBRARIES})

# SDK bridge + ROS node.
add_executable(g1_locomotion_node
  src/locomotion_node.cpp
  src/loco_bridge.cpp
)
add_dependencies(g1_locomotion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(g1_locomotion_node
  g1_loco_core
  ${catkin_LIBRARIES}
  unitree_sdk2
)

install(TARGETS g1_locomotion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_loco_safety test/test_loco_safety.cpp src/loco_safety.cpp)
  add_dependencies(test_loco_safety ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_loco_safety ${catkin_LIBRARIES})

  catkin_add_gtest(test_joint_map test/test_joint_map.cpp src/joint_map.cpp)
  add_dependencies(test_joint_map ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_joint_map ${catkin_LIBRARIES})
endif()
