cmake_minimum_required(VERSION 3.0.2)
project(ros_grpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grpc
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_grpc
  CATKIN_DEPENDS roscpp rospy std_msgs grpc
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

generate_proto(test_action_msg_proto proto/subdir/test_action_msgs.proto)
generate_proto(test_action_proto GRPC proto/test_action.proto)
target_link_libraries(test_action_proto test_action_msg_proto)

generate_proto(stream_proto GRPC proto/stream.proto)
target_link_libraries(stream_proto)

add_library(
  ${PROJECT_NAME} 
  #src/test_action_server_lib.cpp 
  src/stream_server_lib.cpp
  src/stream_client_lib.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GSTREAMER_LIBRARIES})

add_executable(test_action_cpp_server src/test_action_cpp_server.cpp)
target_link_libraries(test_action_cpp_server ${catkin_LIBRARIES} test_action_proto)
add_executable(test_action_async_server src/test_action_async_server.cpp)
target_link_libraries(test_action_async_server ${catkin_LIBRARIES} test_action_proto)

add_executable(test_action_cpp_client src/test_action_cpp_client.cpp)
target_link_libraries(test_action_cpp_client ${catkin_LIBRARIES} test_action_proto)
add_executable(test_action_async_client src/test_action_async_client.cpp)
target_link_libraries(test_action_async_client ${catkin_LIBRARIES} test_action_proto)

#add_executable(test_server src/test_server.cpp)
#add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_server ${PROJECT_NAME} ${catkin_LIBRARIES} test_action_proto)

#### Test
add_executable(test_stream_server test/test_stream_server.cpp)
target_link_libraries(test_stream_server ${PROJECT_NAME} ${catkin_LIBRARIES} stream_proto)
add_executable(test_stream_client test/test_stream_client.cpp)
target_link_libraries(test_stream_client ${PROJECT_NAME} ${catkin_LIBRARIES} stream_proto)
#add_executable(test_action_server_lib src/test_action_server_lib.cpp)
#target_link_libraries(test_action_server_lib ${catkin_LIBRARIES} test_action_proto)