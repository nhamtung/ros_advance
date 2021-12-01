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

namespace ActionServerLib_ns
{
    class ActionServiceImpl final : public Action_grpc::Service {
        Status ActionCallback(ServerContext* context, const ActionRequest* request, ActionReply* reply) override {
            ROS_INFO("test_action_server_lib.cc - action: %d", request->action());
            ROS_INFO("test_action_server_lib.cc - state: %d", request->state());
            ROS_INFO("test_action_server_lib.cc - action_id: %s", request->action_id().c_str());
            ROS_INFO("test_action_server_lib.cc - type: %s", request->type().c_str());
            ROS_INFO("test_action_server_lib.cc - data: %s", request->data().c_str());
            uint32_t action = request->action();
            uint32_t state = request->state();
            if(state == 0){
                for (int8_t i=0; i<action; i++){
                    ROS_INFO("test_action_server_lib.cc - Executed step %d", i+1);
                    sleep(1);
                }
                reply->set_status(3);
                reply->set_message("Executed action done");
                ROS_INFO("test_action_server_lib.cc - SUCCESS - Executed action done!");
            }else if(state == 1){
                uint32_t status = 5;
                reply->set_status(status);
                reply->set_message("Cancel action done");
                ROS_INFO("test_action_server_lib.cc - CANCEL - Cancel action done!");
            }
            return Status::OK;
        }
    };
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