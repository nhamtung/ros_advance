#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/LaserScan.h"

using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
tf2_ros::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
std::shared_ptr<tf2_ros::Buffer> tf_;
std::string odom_frame_id_ = "odom";

void cameraCallback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info){
  ROS_INFO("cameraCallback()");
}
void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan){
  ROS_INFO("laserReceived()");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ros_message_filters");
  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&cameraCallback, _1, _2));
  ROS_INFO("ros_message_filters - Register sync topic /image and topic /camera_info");

  // laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/sick_safetyscanners_front/scan", 100);
  // laser_scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100, nh);
  // laser_scan_filter_->registerCallback(boost::bind(&laserReceived, _1));
  // ROS_INFO("ros_message_filters - Register sync topic /sick_safetyscanners_front/scan and tf");

  ros::spin();
  return 0;
}