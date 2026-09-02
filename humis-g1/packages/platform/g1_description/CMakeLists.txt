cmake_minimum_required(VERSION 3.0.2)
project(g1_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_eigen
  geometry_msgs
  sensor_msgs
  urdf
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_eigen geometry_msgs sensor_msgs urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(base_footprint_publisher src/base_footprint_publisher_node.cpp)
target_link_libraries(base_footprint_publisher ${catkin_LIBRARIES})

add_executable(fake_joint_states src/fake_joint_states_node.cpp)
target_link_libraries(fake_joint_states ${catkin_LIBRARIES})

install(TARGETS base_footprint_publisher fake_joint_states
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/fetch_meshes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY urdf meshes launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_base_footprint test/test_base_footprint.cpp)
  target_include_directories(test_base_footprint PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_tf_tree test/test_tf_tree.test test/test_tf_tree.cpp)
  target_link_libraries(test_tf_tree ${catkin_LIBRARIES})
endif()
