#include "ros_grpc/stream_client_lib.h"

std::string IPC_IP = "";
std::string IPC_PORT = "";

StreamLib_ns::StreamClient* streamGrpc;

bool loadParam(){
	if(!ros::param::get("/IP", IPC_IP)){
		return false;
	}ROS_INFO("IPC_IP: %s", IPC_IP.c_str());
	if(!ros::param::get("/PORT", IPC_PORT)){
		return false;
	}ROS_INFO("IPC_PORT: %s", IPC_PORT.c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_client");
    ros::NodeHandle n;
    ROS_INFO("Create node stream_client"); 
    loadParam();
    grpc_init();
    
    int8_t mode = 1;  // ReplyStream:0, RequestStream:1
    std::string web_server_port = "50051";
    std::string web_server_address = "localhost";
    ROS_INFO("stream_client - web_server_address: %s:%s", web_server_address.c_str(), web_server_port.c_str());
    streamGrpc = new StreamLib_ns::StreamClient(n, web_server_address, web_server_port, mode);

    delete(streamGrpc);
    return 0;
}