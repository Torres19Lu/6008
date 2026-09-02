cmake_minimum_required(VERSION 3.0.2)
project(g1_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  SemanticObject.msg
  SemanticObjectArray.msg
  Detection3D.msg
  Detection3DArray.msg
  NavPoint.msg
  NavPointArray.msg
  LocoStatus.msg
  KeyframeCloud.msg
  KeyframeCloudArray.msg
  KeyframePose.msg
  KeyframePoseArray.msg
  MapManagerState.msg
  MapEntry.msg
  SnapshotEntry.msg
  Gateway.msg
  RelocQuality.msg
)

add_service_files(
  FILES
  SaveMap.srv
  LoadMap.srv
  Relocalize.srv
  AddNavPoint.srv
  QueryNavPoint.srv
  SetMode.srv
  StartMapping.srv
  StartIncremental.srv
  StartLocalization.srv
  StopMapping.srv
  ListMaps.srv
  GetActiveMap.srv
  GetMapNeighbors.srv
  GetTopology.srv
  MapSnapshot.srv
  GatewayCapture.srv
  MapEditCommand.srv
)

add_action_files(
  FILES
  NavigateTo.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs actionlib_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_g1_msgs test/test_g1_msgs.cpp)
  add_dependencies(test_g1_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_include_directories(test_g1_msgs PRIVATE ${catkin_INCLUDE_DIRS})
endif()
