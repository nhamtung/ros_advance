cmake_minimum_required(VERSION 3.0.2)
project(grpc_sample)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Thêm gRPC
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
set(_GRPC_GRPCPP gRPC::grpc++)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grpc_sample
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

function(grpc_generate_cpp PROTO_SRCS PROTO_HDRS GRPC_SRCS GRPC_HDRS proto_file)
    get_filename_component(proto_path "${proto_file}" PATH)
    get_filename_component(proto_name "${proto_file}" NAME_WE)
    set(proto_src "${CMAKE_CURRENT_BINARY_DIR}/${proto_name}.pb.cc")
    set(proto_hdr "${CMAKE_CURRENT_BINARY_DIR}/${proto_name}.pb.h")
    set(grpc_src "${CMAKE_CURRENT_BINARY_DIR}/${proto_name}.grpc.pb.cc")
    set(grpc_hdr "${CMAKE_CURRENT_BINARY_DIR}/${proto_name}.grpc.pb.h")
    add_custom_command(
        OUTPUT "${proto_src}" "${proto_hdr}" "${grpc_src}" "${grpc_hdr}"
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
             --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
             --plugin=protoc-gen-grpc="$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
             -I "${proto_path}"
             "${proto_file}"
        DEPENDS "${proto_file}"
    )
    set(${PROTO_SRCS} "${proto_src}" PARENT_SCOPE)
    set(${PROTO_HDRS} "${proto_hdr}" PARENT_SCOPE)
    set(${GRPC_SRCS} "${grpc_src}" PARENT_SCOPE)
    set(${GRPC_HDRS} "${grpc_hdr}" PARENT_SCOPE)
endfunction()


set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_FILES "${PROTO_SRC_DIR}/grpc_sample.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
grpc_generate_cpp(PROTO_SRCS PROTO_HDRS GRPC_SRCS GRPC_HDRS ${PROTO_FILES})


add_executable(grpc_server_node src/grpc_server.cpp ${PROTO_SRCS} ${GRPC_SRCS})
target_link_libraries(grpc_server_node ${catkin_LIBRARIES} ${Protobuf_LIBRARIES} ${_GRPC_GRPCPP})

add_executable(grpc_client_node test/grpc_client.cpp ${PROTO_SRCS} ${GRPC_SRCS})
target_link_libraries(grpc_client_node ${catkin_LIBRARIES} ${_GRPC_GRPCPP} ${Protobuf_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grpc_sample.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
