cmake_minimum_required(VERSION 3.0.2)
project(ros_pcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rclcpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_pcl
  CATKIN_DEPENDS roscpp rclcpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_pcl test/test_pcl.cpp)
add_dependencies(test_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pcl ${catkin_LIBRARIES})