#include "ros_grpc/stream_server_lib.h"

namespace StreamLib_ns
{
    class StreamImpl final : public StreamGrpc::Service {
        public:
            explicit StreamImpl(ros::NodeHandle n) {}

        private:
            Status ReplyStreamCallback(ServerContext* context, const StreamRequest* rectangle, ServerWriter<StreamReply>* writer) override {
                ROS_INFO("stream_server_lib - ReplyStreamCallback");
                StreamReply feature;
                for (int i=0; i<10; i++) {
                    feature.set_status(response_status);
                    ROS_INFO("stream_server_lib - status: %d", response_status);
                    writer->Write(feature);
                    response_status = response_status+1;
                    if(response_status > 100){
                        response_status=0;
                    }
                }
                return Status::OK;
            }
            int response_status = 0;
    };

    StreamServer::StreamServer(ros::NodeHandle n, std::string address, std::string port){
        ROS_INFO("control_move_lib.cpp -> Init StreamLib()");
        std::string server_address(address + ":" + port);
        StreamImpl service(n);

        ServerBuilder builder;
        builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        builder.RegisterService(&service);
        std::unique_ptr<Server> server(builder.BuildAndStart());
        std::cout << "Server listening on " << server_address << std::endl;
        server->Wait();
    }
    StreamServer::~StreamServer() {}
}