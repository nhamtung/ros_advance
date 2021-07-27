#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

#include "ros_grpc/proto/action.pb.h"
#include "ros_grpc/proto/action.grpc.pb.h"
#include "ros_grpc/proto/subdir/action_msgs.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

class ActionClient {
 public:
  ActionClient(std::shared_ptr<Channel> channel)
      : stub_(Action_grpc::NewStub(channel)) {}

  // Assambles the client's payload, sends it and presents the response back
  // from the server.
  std::string SayHello(const std::string& user) {
    // Data we are sending to the server.
    ActionRequest request;
    // request.set_type(user);
    request.set_data(user);

    // Container for the data we expect from the server.
    ActionReply reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->ExecuteAction(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      return reply.message();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<Action_grpc::Stub> stub_;
};

int main(int argc, char** argv) {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint (in this case,
  // localhost at port 50051). We indicate that the channel isn't authenticated
  // (use of InsecureChannelCredentials()).
  grpc_init();
  ActionClient action(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));
  std::string user("TungNV");
  std::string reply = action.SayHello(user);
  std::cout << "Action received: " << reply << std::endl;

  return 0;
}