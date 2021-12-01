- Reference link: https://github.com/CogRob/catkin_grpc_examples
- Reference link: https://github.com/chungphb/grpc-cpp

# Require 
- Ubuntu 18.04 Bionic
- ROS Melodic
- Install ROS grpc: $sudo apt-get install ros-melodic-grpc

# Test
- Run example:
    + Direct to workspace
    + Build: $catkin_make
    + Source: $source devel/setup.bash
    + $roslaunch ros_grpc test_action_cpp_server.launch
    + $roslaunch ros_grpc test_action_cpp_client.launch

- Run example async:
    + Direct to workspace
    + Build: $catkin_make
    + Source: $source devel/setup.bash
    + $roslaunch ros_grpc test_action_async_server.launch
    + $roslaunch ros_grpc test_action_async_client.launch

- Run server OPP:
    + Direct to workspace
    + Build: $catkin_make
    + $roslaunch ros_grpc test_server.launch
    + $roslaunch ros_grpc test_action_cpp_client.launch