#ifndef STREAM_CLIENT_LIB_H
#define STREAM_CLIENT_LIB_H
#pragma once
#pragma comment(lib, "urlmon.lib")

#include <ros/ros.h>
#include <grpc++/grpc++.h>

#include "ros_grpc/proto/stream.pb.h"
#include "ros_grpc/proto/stream.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using catkin_grpc::ros_grpc::StreamGrpc;
using catkin_grpc::ros_grpc::StreamRequest;
using catkin_grpc::ros_grpc::StreamReply;

namespace StreamLib_ns
{
    class StreamClient{
        public:
            StreamClient(ros::NodeHandle n, std::string address, std::string port);
            ~StreamClient();
    };
}

#endif