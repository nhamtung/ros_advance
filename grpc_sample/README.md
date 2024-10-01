
# Require 
- Ubuntu 20.04
- ROS Noetic

- Install dependency package
    + Directory to workspace
    + $sudo chmod 777 ./src/ros_advance/grpc_sample/installation.sh
    + $./src/ros_advance/grpc_sample/installation.sh

# Test
- Run example:
    + Direct to workspace
    + Build: $catkin_make
    + Source: $source devel/setup.bash
    + $roslaunch grpc_sample grpc_server.launch
    + $roslaunch grpc_sample grpc_client.launch