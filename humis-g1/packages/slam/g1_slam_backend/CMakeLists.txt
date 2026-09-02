cmake_minimum_required(VERSION 3.0.2)
project(g1_slam_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  eigen_conversions
  message_filters
  g1_msgs
  std_srvs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
    tf2 tf2_ros tf2_eigen tf2_geometry_msgs
    pcl_ros pcl_conversions eigen_conversions message_filters g1_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# Backend core: ROS-free logic (pose graph now; scan_context + backend later),
# linked by both the node (later) and the unit tests.
add_library(${PROJECT_NAME}_core
  src/pose_graph.cpp
  src/scan_context.cpp
  src/backend.cpp
  src/tracking_correction.cpp
)
target_link_libraries(${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/backend_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Provenance of the resolved GTSAM (two installs exist on this host:
  # /usr/lib/cmake/GTSAM and /opt/ros/noetic). Printed so the build log records
  # which one (and its Eigen handling) the ABI smoke test actually links.
  message(STATUS "g1_slam_backend: GTSAM_DIR=${GTSAM_DIR} GTSAM_VERSION=${GTSAM_VERSION}")
  message(STATUS "g1_slam_backend: GTSAM_INCLUDE_DIR=${GTSAM_INCLUDE_DIR}")
  message(STATUS "g1_slam_backend: GTSAM_USE_SYSTEM_EIGEN=${GTSAM_USE_SYSTEM_EIGEN}")

  catkin_add_gtest(test_gtsam_abi test/test_gtsam_abi.cpp)
  target_include_directories(test_gtsam_abi PRIVATE
    ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
  target_link_libraries(test_gtsam_abi gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_pose_graph test/test_pose_graph.cpp)
  target_link_libraries(test_pose_graph ${PROJECT_NAME}_core gtsam ${catkin_LIBRARIES})

  catkin_add_gtest(test_scan_context test/test_scan_context.cpp)
  target_link_libraries(test_scan_context ${PROJECT_NAME}_core ${catkin_LIBRARIES})

  catkin_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_map_io test/test_map_io.cpp)
  target_link_libraries(test_map_io ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_relocalize test/test_relocalize.cpp)
  target_link_libraries(test_relocalize ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_reloc_quality test/test_reloc_quality.cpp)
  target_include_directories(test_reloc_quality PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_reloc_quality ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_incremental test/test_incremental.cpp)
  target_link_libraries(test_incremental ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_tracking_correction test/test_tracking_correction.cpp)
  target_link_libraries(test_tracking_correction ${PROJECT_NAME}_core ${catkin_LIBRARIES})

  catkin_add_gtest(test_track_observability test/test_track_observability.cpp)
  target_link_libraries(test_track_observability ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  # Hardware-free launch + config contract checks (needs python3-nose).
  catkin_add_nosetests(test/test_slam_backend_launch.py)

  # Node-level rostest for the map-management hooks: the read-only guard,
  # /slam/keyframe_poses, and the begin_incremental/reset services. The fixture map
  # is built in-process with the core; the running node's services are exercised.
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_backend_node_hooks
    test/backend_node_hooks.test
    test/backend_node_hooks.cpp)
  target_link_libraries(test_backend_node_hooks
    ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

  # Node-level closed-loop rostest: in a well-conditioned scene with a drifting
  # odom, the robust tracking correction must keep map->odom tracking the true
  # offset (regression guard that the projection/clamp/low-pass/jump-reject do not
  # break NORMAL localization).
  add_rostest_gtest(test_tracking_closed_loop
    test/tracking_closed_loop.test
    test/tracking_closed_loop.cpp)
  target_link_libraries(test_tracking_closed_loop
    ${PROJECT_NAME}_core gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})
endif()
