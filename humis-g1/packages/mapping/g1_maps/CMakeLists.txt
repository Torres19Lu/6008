cmake_minimum_required(VERSION 3.0.2)
project(g1_maps)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY maps/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)
# topology/ holds runtime data (not committed); install it only when present so a fresh
# clone still configures. The manager treats an absent topology.yaml as an empty graph
# and writes it on the first gateway save.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/topology")
  install(DIRECTORY topology/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/topology)
endif()
install(PROGRAMS scripts/fetch_maps.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
