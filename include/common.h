#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "System.h"
#include "ImuTypes.h"
#include "Converter.h"
#include "SerializationUtils.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Atlas.h"
#include "Settings.h"
#include "yaml-cpp/yaml.h"


extern ros::Publisher pose_pub;
extern ros::Publisher pose_twist_pub;
extern ros::Publisher map_points_pub;
extern image_transport::Publisher rendered_image_pub;

extern std::string map_frame_id, pose_frame_id;

void setup_ros_publishers(ros::NodeHandle &node_handler, image_transport::ImageTransport &image_transport);
void setup_tf_orb_to_ros(ORB_SLAM3::System::eSensor);


void publish_ros_pose_tf(cv::Mat, cv::Mat, ros::Time, ros::Time, ORB_SLAM3::System::eSensor, double, double);
void publish_tf_transform(tf::Transform, ros::Time);
void publish_pose_stamped(tf::Transform, ros::Time);
void publish_ros_tracking_img(cv::Mat, ros::Time);
void publish_ros_tracking_mappoints(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);
void publish_odom_stamped(tf::Transform, cv::Mat, cv::Mat, ros::Time, ros::Time, double, double);

tf::Transform from_orb_to_ros_tf_transform(cv::Mat);
sensor_msgs::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);

void set_vel_bound(ORB_SLAM3::System *, double, double);