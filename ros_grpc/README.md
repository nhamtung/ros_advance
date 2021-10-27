- Reference link: https://github.com/CogRob/catkin_grpc_examples

# Require 
- Ubuntu 18.04 Bionic
- ROS Melodic
- Install ROS grpc: $sudo apt-get install ros-melodic-grpc

- Run example:
    + Direct to workspace
    + Build: $catkin_make
    + Source: $source devel/setup.bash
    + $roslaunch ros_grpc action_cpp_server.launch
    + $roslaunch ros_grpc action_cpp_client.launch