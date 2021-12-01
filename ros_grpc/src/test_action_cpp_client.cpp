#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

#include "ros_grpc/proto/test_action.pb.h"
#include "ros_grpc/proto/test_action.grpc.pb.h"
#include "ros_grpc/proto/subdir/test_action_msgs.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

class ActionClient {
 public:
  ActionClient(std::shared_ptr<Channel> channel):stub_(Action_grpc::NewStub(channel)) {}
  
  ActionReply callAction(int32_t action, int32_t state, const std::string& action_id, const std::string& type, const std::string& data) {
    ActionRequest request;
    request.set_action(action);
    request.set_state(state);
    request.set_action_id(action_id);
    request.set_type(type);
    request.set_data(data);

    ActionReply reply;
    ClientContext context;
    
    Status status = stub_->ActionCallback(&context, request, &reply);  // The actual RPC.

    // Act upon its status.
    if (!status.ok()) {
      std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    ROS_INFO("action_cpp_client.cpp - status: %d - message: %s", reply.status(), reply.message().c_str());
    return reply;
  }

 private:
  std::unique_ptr<Action_grpc::Stub> stub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_cpp_client");
  grpc_init();
  ActionClient Action(grpc::CreateChannel("localhost:50051", grpc::InsecureChannelCredentials()));

  int32_t action = 5;
  int32_t state = 0;
  std::string action_id = "0000-0000-0000-0000";
  std::string type = "std_msgs/String";
  std::string data = "Action";    
  ActionReply reply = Action.callAction(action, state, action_id, type, data);

  ros::spin();
  return 0;
}