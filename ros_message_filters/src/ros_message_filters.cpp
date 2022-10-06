#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sick_lidar_localization/LocalizationControllerResultMessage0502.h>
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
using namespace sick_lidar_localization;
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
void callback(const LocalizationControllerResultMessage0502ConstPtr& lidar_loc_sub, const TwistConstPtr& velocity_sub){
  ROS_INFO("callback()");
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

  // message_filters::Subscriber<LocalizationControllerResultMessage0502> lidar_loc_sub(nh, "/localizationcontroller/out/localizationcontroller_result_message_0502", 1);
  // message_filters::Subscriber<Twist> velocity_sub(nh, "/cmd_vel", 10);
  // TimeSynchronizer<LocalizationControllerResultMessage0502, Twist> sync(lidar_loc_sub, velocity_sub, 10);
  // sync.registerCallback(boost::bind(&callback, _1, _2));
  // ROS_INFO("ros_message_filters - Register sync topic /localizationcontroller/out/localizationcontroller_result_message_0502 and topic /cmd_vel");

  ros::spin();
  return 0;
}