#include <ros_grpc/stream_client_lib.h>

namespace StreamLib_ns
{
    class StreamImpl {
        public:
            StreamImpl(std::shared_ptr<Channel> channel) : stub_(StreamGrpc::NewStub(channel)) {}

            void ReplyStream() {
                StreamRequest rect;
                StreamReply feature;
                ClientContext context;
                std::unique_ptr<ClientReader<StreamReply> > reader(stub_->ReplyStreamCallback(&context, rect));
                while (reader->Read(&feature)) {
                    ROS_INFO("stream_client_lib - Read: %d", feature.status());
                }
                Status status = reader->Finish();
                if (status.ok()) {
                    std::cout << "ListFeatures rpc succeeded." << std::endl;
                } else {
                    std::cout << "ListFeatures rpc failed." << std::endl;
                }
            }

        private:
            const float kCoordFactor_ = 10000000.0;
            std::unique_ptr<StreamGrpc::Stub> stub_;
            std::vector<StreamReply> feature_list_;
    };
    StreamClient::StreamClient(ros::NodeHandle n, std::string address, std::string port){
        ROS_INFO("stream_client_lib -> Init StreamLib()");
        std::string server_address(address + ":" + port);
        StreamImpl guide(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
        guide.ReplyStream();
    }
    StreamClient::~StreamClient() {}
}