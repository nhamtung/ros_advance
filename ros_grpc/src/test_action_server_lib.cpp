#include "ros_grpc/test_action_server_lib.h"

namespace ActionServerLib_ns
{

    ActionServerLib::ActionServerLib(ros::NodeHandle n){
        ROS_INFO("test_action_server_lib.cpp -> Init ActionServerLib()");

        // Registration to publish the topic
        pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 
        ROS_INFO("test_action_server_lib.cpp - Publish topic /cmd_vel");
        
        std::string server_address("localhost:50051");
        ActionServiceImpl service;

        // ServerBuilder builder;
        // // Listen on the given address without any authentication mechanism.
        // builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        // // Register "service" as the instance through which we'll communicate with
        // // clients. In this case it corresponds to an *synchronous* service.
        // builder.RegisterService(&service);
        // // Finally assemble the server.
        // std::unique_ptr<Server> server(builder.BuildAndStart());
        // std::cout << "Server listening on " << server_address << std::endl;

        // // Wait for the server to shutdown. Note that some other thread must be
        // // responsible for shutting down the server for this call to ever return.
        // server->Wait();
    }
    ActionServerLib::~ActionServerLib() {}


    // Publish the topic
    void ActionServerLib::pubCmdvel(geometry_msgs::Twist cmd_vel_){
        pub_cmd_vel.publish(cmd_vel_);
        ROS_INFO("execute_action_lib.cpp - Publish to topic /cmd_vel");
    }
    void ActionServerLib::stopRobot(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        pub_cmd_vel.publish(cmd_vel);
        ROS_INFO("execute_action_lib.cpp - STOP ROBOT");
    }
}