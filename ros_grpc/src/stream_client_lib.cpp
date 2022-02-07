#include <ros_grpc/stream_client_lib.h>

namespace StreamLib_ns
{
    class StreamImpl {
        public:
            StreamImpl(std::shared_ptr<Channel> channel) : stub_(StreamGrpc::NewStub(channel)) {}

            void ReplyStream() {
                StreamRequest request;
                StreamReply relay;
                ClientContext context;
                std::unique_ptr<ClientReader<StreamReply> > reader(stub_->ReplyStreamCallback(&context, request));
                while (reader->Read(&relay)) {
                    ROS_INFO("stream_client_lib - Read: %d", relay.status());
                }
                Status status = reader->Finish();
                if (status.ok()) {
                    std::cout << "ListFeatures rpc succeeded." << std::endl;
                } else {
                    std::cout << "ListFeatures rpc failed." << std::endl;
                }
            }
            std::vector<StreamRequest> stream_request_list_;
            void RequestStream() {
                StreamRequest request;
                StreamReply relay;
                ClientContext context;
                std::unique_ptr<ClientWriter<StreamRequest> > writer(stub_->RequestStreamCallback(&context, &relay));
                for (int i = 0; i < 10; i++) {
                    request.set_data(i);
                    if (!writer->Write(request)) {
                        break;
                    }
                    ros::Duration(0.5).sleep();
                }
                writer->WritesDone();
                Status status = writer->Finish();
                if (status.ok()) {
                    std::cout << "Finished trip" << std::endl;
                } else {
                    std::cout << "RecordRoute rpc failed." << std::endl;
                }
            }
        private:
            std::unique_ptr<StreamGrpc::Stub> stub_;
    };
    StreamClient::StreamClient(ros::NodeHandle n, std::string address, std::string port, int8_t mode){
        ROS_INFO("stream_client_lib -> Init StreamLib()");
        std::string server_address(address + ":" + port);
        StreamImpl guide(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
        if (mode == 0){
            guide.ReplyStream();
        }else if(mode == 1){
            guide.RequestStream();
        }
    }
    StreamClient::~StreamClient() {}
}