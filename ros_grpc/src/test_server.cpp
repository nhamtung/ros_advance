#include "ros/ros.h"
#include <grpc++/grpc++.h>
#include "ros_grpc/test_action_server_lib.h"

ActionServerLib_ns::ActionServerLib *actionServer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_server");
    ros::NodeHandle n;
    ROS_INFO("test_server.cpp - Create node test_server"); 
    
    // grpc_init();
    // actionServer = new ActionServerLib_ns::ActionServerLib(n);
    
    ros::spin();
    // delete(actionServer);
    return 0;
}