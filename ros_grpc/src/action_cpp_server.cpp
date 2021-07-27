#include "ros/ros.h"

#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

#include "ros_grpc/proto/action.pb.h"
#include "ros_grpc/proto/action.grpc.pb.h"
#include "ros_grpc/proto/subdir/action_msgs.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

// Logic and data behind the server's behavior.
class ActionServiceImpl final : public Action_grpc::Service {
  Status SayHello(ServerContext* context, const ActionRequest* request, ActionReply* reply) override {
    ROS_INFO("action_cpp_server.cc - Received a request from Client");
    std::string prefix("Hello ");
    reply->set_message(prefix + request->name());
    return Status::OK;
  }
};

void RunServer() {
  std::string server_address("0.0.0.0:50051");
  ActionServiceImpl service;

  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  grpc_init();
  RunServer();

  return 0;
}
