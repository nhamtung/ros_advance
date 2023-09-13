#include <boost/thread.hpp>
#include "ros_grpc/stream_server_lib.h"

std::string IPC_IP = "";
std::string IPC_PORT = "";

boost::thread* grpc_server_thread;
StreamLib_ns::StreamServer* streamGrpc;

bool loadParam(){
	if(!ros::param::get("/IP", IPC_IP)){
		return false;
	}ROS_INFO("IPC_IP: %s", IPC_IP.c_str());
	if(!ros::param::get("/PORT", IPC_PORT)){
		return false;
	}ROS_INFO("IPC_PORT: %s", IPC_PORT.c_str());
    return true;
}

void grpcServerThread(ros::NodeHandle n, std::string web_server_address, std::string port){
    streamGrpc = new StreamLib_ns::StreamServer(n, web_server_address, port);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_server");
    ros::NodeHandle n;
    ROS_INFO("Create node stream_server"); 
    loadParam();
    
    std::string web_server_address = "localhost";
    ROS_INFO("control_move_lib - web_server_address: %s", web_server_address.c_str());
    grpc_server_thread = new boost::thread (&grpcServerThread, n, web_server_address, "50051");

    ros::spin();
    delete(streamGrpc);
    return 0;
}