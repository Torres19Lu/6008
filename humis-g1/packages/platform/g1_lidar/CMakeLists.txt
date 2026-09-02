cmake_minimum_required(VERSION 3.0.2)
project(g1_lidar)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/generate_livox_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The log filter is referenced by lidar.launch as $(find g1_lidar)/scripts/...,
# so install it under the package scripts dir (not bin) to keep that path valid.
install(PROGRAMS
  scripts/filter_node_log.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install the tracked template only; the generated MID360_config.json is runtime
# state and must never be installed or committed.
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "MID360_config.json" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_generate_livox_config.py)
  catkin_add_nosetests(test/test_lidar_launch.py)
endif()
