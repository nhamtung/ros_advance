#ifndef STREAM_SERVER_LIB_H
#define STREAM_SERVER_LIB_H
#pragma once
#pragma comment(lib, "urlmon.lib")

#include <ros/ros.h>
#include <grpc++/grpc++.h>

#include "ros_grpc/proto/stream.pb.h"
#include "ros_grpc/proto/stream.grpc.pb.h"

using namespace std;

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;

using catkin_grpc::ros_grpc::StreamGrpc;
using catkin_grpc::ros_grpc::StreamRequest;
using catkin_grpc::ros_grpc::StreamReply;

namespace StreamLib_ns
{
    class StreamServer{
        public:
            StreamServer(ros::NodeHandle n, std::string address, std::string port);
            ~StreamServer();
    };
}

#endif