#include"../../../include/common.h"

using namespace std;

double max_lin_vel_;
double max_ang_vel_;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb):mpSLAM(pSLAM), mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void SyncWithImu();

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD-IMU");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");
    node_handler.param<double>(node_name + "/fetch/max_lin_vel", max_lin_vel_, 1);
    node_handler.param<double>(node_name + "/fetch/max_ang_vel", max_ang_vel_, 2.5);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_RGBD, true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth_registered/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node_handler, "/imu", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub, imu_sub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped> ("/orb_slam3_ros/camera", 1);

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);

    setup_tf_orb_to_ros(ORB_SLAM3::System::RGBD);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Main algorithm runs here
    cv::Mat Tcw_prev = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec()).matrix());
    ros::Time previous_frame_time = ros::Time::now();
    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec()).matrix());
    ros::Time current_frame_time = ros::Time::now();

    publish_ros_pose_tf(Tcw, Tcw_prev, previous_frame_time, current_frame_time, ORB_SLAM3::System::STEREO, max_lin_vel_, max_ang_vel_);

    publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

    // publish_ros_tracking_img(mpSLAM->GetCurrentFrame(), current_frame_time);
}