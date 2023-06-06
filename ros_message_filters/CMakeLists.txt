cmake_minimum_required(VERSION 3.0.2)
project(ros_message_filters)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf2_ros
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_message_filters
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_filters nav_msgs sensor_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_message_filters src/ros_message_filters.cpp)
add_dependencies(ros_message_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_message_filters ${catkin_LIBRARIES})

#### Test
#add_executable(test_move_client test/test_move_client.cpp)
#target_link_libraries(test_move_client ${PROJECT_NAME} ${catkin_LIBRARIES})