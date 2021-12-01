#include <grpc++/grpc++.h>
#include <memory>
#include <iostream>

#include "ros_grpc/proto/test_action.pb.h"
#include "ros_grpc/proto/test_action.grpc.pb.h"
#include "ros_grpc/proto/subdir/test_action_msgs.pb.h"

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using catkin_grpc::ros_grpc::ActionRequest;
using catkin_grpc::ros_grpc::ActionReply;
using catkin_grpc::ros_grpc::Action_grpc;

class ServerImpl final {
public:
    ~ServerImpl() {
        _server->Shutdown();
        _queue->Shutdown();
    }

    void Run() {
        std::string server_address{"localhost:2511"};

        // Build server
        ServerBuilder builder;
        builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        builder.RegisterService(&_service);
        _queue = builder.AddCompletionQueue();
        _server = builder.BuildAndStart();
        
        // Run server
        std::cout << "Server listening on " << server_address << std::endl;
        HandleRPCs();
    }

private:
    class CallData {
    public:
        CallData(Action_grpc::AsyncService* service, ServerCompletionQueue* queue) : _service{service}, _queue{queue}, _responder{&_context}, _status{CallStatus::CREATE} {
            Proceed();
        }

        void Proceed() {
            switch (_status) {
                case CallStatus::CREATE: {
                    _status = CallStatus::PROCESS;
                    _service->RequestActionCallback(&_context, &_request, &_responder, _queue, _queue, this);
                    break;
                }
                case CallStatus::PROCESS: {
                    new CallData{_service, _queue};
                    _response.set_message("Hello " + _request.action());
                    _status = CallStatus::FINISH;
                    _responder.Finish(_response, Status::OK, this);
                    break;
                }
                default: {
                    delete this;
                }
            }
        }

    private:
        Action_grpc::AsyncService* _service;
        ServerCompletionQueue* _queue;
        ServerContext _context;
        ActionRequest _request;
        ActionReply _response;
        ServerAsyncResponseWriter<ActionReply> _responder;
        enum class CallStatus {
            CREATE, PROCESS, FINISH
        };
        CallStatus _status;
    };

    void HandleRPCs() {
        new CallData{&_service, _queue.get()};
        void* tag;
        bool ok;
        while (true) {
            if (_queue->Next(&tag, &ok) && ok) {
                static_cast<CallData*>(tag)->Proceed();
            } else {
                std::cerr << "Something went wrong" << std::endl;
                abort();
            }
        }
    }

    Action_grpc::AsyncService _service;
    std::unique_ptr<ServerCompletionQueue> _queue;
    std::unique_ptr<Server> _server;
};

int main(int argc, char** argv) {
    ServerImpl server;
    server.Run();
    return 0;
}