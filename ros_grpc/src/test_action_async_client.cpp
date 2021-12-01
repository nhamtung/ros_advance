#include <grpc++/grpc++.h>
#include <memory>
#include <thread>
#include <iostream>

#include "ros_grpc/proto/test_action.pb.h"
#include "ros_grpc/proto/test_action.grpc.pb.h"
#include "ros_grpc/proto/subdir/test_action_msgs.pb.h"

using grpc::Channel;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

namespace ns1 {
    class SampleClient {
    public:
        SampleClient(std::shared_ptr<Channel> channel) : _stub{Action_grpc::NewStub(channel)} {}

        std::string ActionCallback(const std::string& request_sample_field) {
            // Prepare request
            ActionRequest request;
            request.set_action_id(request_sample_field);

            // Create an RPC object
            ActionReply response;
            ClientContext context;
            CompletionQueue queue;
            Status status;
            std::unique_ptr<ClientAsyncResponseReader<ActionReply>> rpc;
            rpc = _stub->PrepareAsyncActionCallback(&context, request, &queue);
            
            // Initiate the RPC call
            rpc->StartCall();
            
            // Request to update the server's response and the call status upon completion of the RPC
            rpc->Finish(&response, &status, (void*)1);

            // Complete the RPC call
            void* tag;
            bool ok = false;
            if (queue.Next(&tag, &ok) && ok && tag == (void*)1) {
                if (status.ok()) {
                    return response.message();
                } else {
                    std::cerr << status.error_code() << ": " << status.error_message() << std::endl;
                    return "RPC failed";
                }
            } else {
                std::cerr << "Something went wrong" << std::endl;
                abort();
            }
        }
    private:
        std::unique_ptr<Action_grpc::Stub> _stub;
    };

    void RunClient() {
        std::string server_address{"localhost:2511"};
        SampleClient client{grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials())};
        std::string request_sample_field{"world"};
        std::string response_sample_field = client.ActionCallback(request_sample_field);
        std::cout << "Client received: " << response_sample_field << std::endl;
    }
}

namespace ns2 {
    class SampleClient {
    public:
        SampleClient(std::shared_ptr<Channel> channel) : _stub{Action_grpc::NewStub(channel)} {}

        void ActionCallback(const std::string& request_sample_field) {
            // Prepare request
            ActionRequest request;
            request.set_action_id(request_sample_field);

            // Create an AsyncClientCall object to store RPC data
            auto* call = new AsyncClientCall;
            
            // Create an RPC object
            call->rpc = _stub->PrepareAsyncActionCallback(&call->context, request, &_queue);
            
            // Initiate the RPC call
            call->rpc->StartCall();
            
            // Request to update the server's response and the call status upon completion of the RPC
            call->rpc->Finish(&call->response, &call->status, (void*)call);
        }

        void AsyncCompleteRPC() {
            void* tag;
            bool ok = false;
            while (_queue.Next(&tag, &ok)) {
                if (!ok) {
                    std::cerr << "Something went wrong" << std::endl;
                    abort();
                }
                std::string err;
                auto* call = static_cast<AsyncClientCall*>(tag);
                if (call) {
                    if (call->status.ok()) {
                        std::cout << "Client received: " << call->response.message() << std::endl;
                    } else {
                        std::cerr << call->status.error_code() << ": " << call->status.error_message() << std::endl;
                        std::cout << "Client received: RPC failed" << std::endl;
                    }
                } else {
                    err = "A client call was deleted";
                }
                delete call;
                if (!err.empty()) {
                    throw std::runtime_error(err);
                }
            }
        }

    private:
        struct AsyncClientCall {
            ActionReply response;
            ClientContext context;
            Status status;
            std::unique_ptr<ClientAsyncResponseReader<ActionReply>> rpc;
        };
        std::unique_ptr<Action_grpc::Stub> _stub;
        CompletionQueue _queue;
    };

    void RunClient() {
        std::string server_address{"localhost:2511"};
        SampleClient client{grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials())};
        std::thread thread{&SampleClient::AsyncCompleteRPC, &client};
        for (int i = 0; i < 100; ++i) {
            std::string request_sample_field{"world " + std::to_string(i)};
            client.ActionCallback(request_sample_field);
        }
        std::cout << "Press Ctrl + C to quit..." << std::endl << std::endl;
        thread.join();
    }
}

int main(int argc, char** argv) {
    ns1::RunClient();
    ns2::RunClient();
    return 0;
}