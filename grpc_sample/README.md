
# Require 
- Ubuntu 20.04
- ROS Noetic

# Test
- Run example:
    + Direct to workspace
    + Build: $catkin_make
    + Source: $source devel/setup.bash
    + $roslaunch grpc_sample grpc_server.launch
    + $roslaunch grpc_sample grpc_client.launch