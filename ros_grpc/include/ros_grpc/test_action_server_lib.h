#ifndef TEST_ACTION_SERVER_LIB_H
#define TEST_ACTION_SERVER_LIB_H
#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

#include "ros_grpc/proto/test_action.pb.h"
#include "ros_grpc/proto/test_action.grpc.pb.h"
#include "ros_grpc/proto/subdir/test_action_msgs.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

using namespace std;

namespace ActionServerLib_ns
{
    class ActionServerLib
    {
        private:
            ros::Publisher pub_cmd_vel;
        public:
            ActionServerLib(ros::NodeHandle n);
            ~ActionServerLib();

            bool isEnableAction = true;
            
            void pubCmdvel(geometry_msgs::Twist cmd_vel_);
            void stopRobot();
    };
}
#endif