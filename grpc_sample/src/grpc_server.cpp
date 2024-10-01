#include <ros/ros.h>
#include <grpcpp/grpcpp.h>
#include "grpc_sample.grpc.pb.h"  // Include file được tạo từ protoc
#include "grpc_sample.pb.h"


static bool server_running = true;


// Server Implementation
class ExampleServiceImpl final : public grpc_sample::ExampleService::Service {
    grpc::Status SayHello(grpc::ServerContext* context, const grpc_sample::HelloRequest* request, grpc_sample::HelloReply* reply) override {
        ROS_INFO("[grpc_server] Receive request");
        std::string prefix("Hello, ");
        reply->set_message(prefix + request->name());
        return grpc::Status::OK;
    }
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    ExampleServiceImpl service;

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    ROS_INFO("Server listening on %s", server_address.c_str());
    // server->Wait();

    ros::Rate loop_rate(20);
    while(ros::ok() && server_running) {
        // Bạn có thể thêm mã khác ở đây nếu cần
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Đợi một chút
        loop_rate.sleep();
        ros::spinOnce();

    }
    server->Shutdown();
    std::cout << "Server stopped." << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grpc_server_node");

    ros::NodeHandle nh;
    ROS_INFO("Starting gRPC server...");
    
    // Chạy gRPC server trong ROS node
    RunServer();

    ros::spin();
    server_running = false;
    return 0;
}
