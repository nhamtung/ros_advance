#include <ros/ros.h>
#include <grpcpp/grpcpp.h>
#include "grpc_sample.grpc.pb.h"  // Include file từ protoc

void RunClient() {
    std::string target_str = "localhost:50051";
    auto channel = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
    std::unique_ptr<grpc_sample::ExampleService::Stub> stub_ = grpc_sample::ExampleService::NewStub(channel);

    // Tạo request
    grpc_sample::HelloRequest request;
    request.set_name("TungNV");

    // Tạo response
    grpc_sample::HelloReply reply;
    grpc::ClientContext context;

    // Gửi request đến server
    grpc::Status status = stub_->SayHello(&context, request, &reply);

    if (status.ok()) {
        ROS_INFO("Client received: %s", reply.message().c_str());
    } else {
        ROS_ERROR("gRPC call failed");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grpc_client_node");

    ros::NodeHandle nh;
    ROS_INFO("Starting gRPC client...");

    // Chạy gRPC client trong ROS node
    RunClient();

    ros::spin();
    return 0;
}
