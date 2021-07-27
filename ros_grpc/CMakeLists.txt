cmake_minimum_required(VERSION 3.0.2)
project(ros_grpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grpc
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_grpc
  CATKIN_DEPENDS roscpp rospy std_msgs grpc
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

generate_proto(action_msg_proto proto/subdir/action_msgs.proto)
generate_proto(action_proto GRPC proto/action.proto)
target_link_libraries(action_proto action_msg_proto)

add_executable(action_cpp_server src/action_cpp_server.cpp)
target_link_libraries(action_cpp_server ${catkin_LIBRARIES} action_proto)

add_executable(action_cpp_client src/action_cpp_client.cpp)
target_link_libraries(action_cpp_client ${catkin_LIBRARIES} action_proto)

#catkin_python_setup()