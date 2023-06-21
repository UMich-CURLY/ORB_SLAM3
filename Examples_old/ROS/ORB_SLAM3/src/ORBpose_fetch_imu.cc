#include"../../../include/common.h"

using namespace std;

double max_lin_vel_;
double max_ang_vel_;
string traj_filename;

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

    void GrabRGB(const sensor_msgs::ImageConstPtr& msgRGB);
    void GrabD(const sensor_msgs::ImageConstPtr& msgD);
    // cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    std::tuple<cv::Mat, cv::Mat> GetImageRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    void SyncWithImu();
    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutexRGB,mBufMutexD;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    // const bool bmClahe;
    // cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
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
    node_handler.param<std::string>(node_name + "/traj_filename", traj_filename, "orb_slam3.txt");
    node_handler.param<double>(node_name + "/fetch/max_lin_vel", max_lin_vel_, 1);
    node_handler.param<double>(node_name + "/fetch/max_ang_vel", max_ang_vel_, 2.5);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_RGBD, true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    ros::Subscriber imu_sub = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber rgb_sub = node_handler.subscribe("/camera/rgb/image_raw", 100, &ImageGrabber::GrabRGB, &igb);
    ros::Subscriber depth_sub = node_handler.subscribe("/camera/depth_registered/image_raw", 100, &ImageGrabber::GrabD, &igb);

    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped> ("/orb_slam3_ros/camera", 1);

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);

    setup_tf_orb_to_ros(ORB_SLAM3::System::IMU_RGBD);
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("/home/jonathan/"+traj_filename);

    ros::shutdown();

    return 0;
}

// void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptrRGB;
//     try
//     {
//         cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     cv_bridge::CvImageConstPtr cv_ptrD;
//     try
//     {
//         cv_ptrD = cv_bridge::toCvShare(msgD);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
    
//     // Main algorithm runs here
//     cv::Mat Tcw_prev = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec()).matrix());
//     ros::Time previous_frame_time = ros::Time::now();
//     cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec()).matrix());
//     ros::Time current_frame_time = ros::Time::now();

//     publish_ros_pose_tf(Tcw, Tcw_prev, previous_frame_time, current_frame_time, ORB_SLAM3::System::IMU_RGBD, max_lin_vel_, max_ang_vel_);

//     publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

//     // publish_ros_tracking_img(mpSLAM->GetCurrentFrame(), current_frame_time);
// }

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}

void ImageGrabber::GrabRGB(const sensor_msgs::ImageConstPtr &msgRGB)
{
    mBufMutexRGB.lock();
    if (!imgRGBBuf.empty()) imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);
    mBufMutexRGB.unlock();
}

void ImageGrabber::GrabD(const sensor_msgs::ImageConstPtr &msgD)
{
    mBufMutexD.lock();
    if (!imgDBuf.empty()) imgDBuf.pop();
    imgDBuf.push(msgD);
    mBufMutexD.unlock();
}

std::tuple<cv::Mat, cv::Mat> ImageGrabber::GetImageRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
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
        return std::make_tuple(cv::Mat(), cv::Mat());
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return std::make_tuple(cv::Mat(), cv::Mat());
    }

    return std::make_tuple(cv_ptrRGB->image.clone(), cv_ptrD->image.clone());
}

// cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
// {
//   // Copy the ros image message to cv::Mat.
//   cv_bridge::CvImageConstPtr cv_ptr;
//   try
//   {
//     // cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
//     cv_ptr = cv_bridge::toCvShare(img_msg);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }
  
//   if(cv_ptr->image.type()==0)
//   {
//     return cv_ptr->image.clone();
//   }
//   else
//   {
//     std::cout << "Error type" << std::endl;
//     return cv_ptr->image.clone();
//   }
// }

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1) 
    {
        cv::Mat imRGB, imD;
        double tImRGB=0.0, tImD=0.0;
        if (!imgRGBBuf.empty() && !imgDBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            ROS_INFO("Syncing with IMU");
            tImRGB = imgRGBBuf.front()->header.stamp.toSec();
            tImD   = imgDBuf.front()->header.stamp.toSec();

            this->mBufMutexD.lock();
            while((tImRGB-tImD)>maxTimeDiff && imgDBuf.size()>1)
            {
                imgDBuf.pop();
                tImD = imgDBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexD.unlock();

            this->mBufMutexRGB.lock();
            while((tImD-tImRGB)>maxTimeDiff && imgRGBBuf.size()>1)
            {
                imgRGBBuf.pop();
                tImRGB = imgRGBBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRGB.unlock();
            ROS_INFO("Done Syncing with IMU");

            if((tImRGB-tImD)>maxTimeDiff || (tImD-tImRGB)>maxTimeDiff)
            {
                std::cout << "big time difference: " << tImRGB - tImD << std::endl;
                continue;
            }

            if(tImRGB>mpImuGb->imuBuf.back()->header.stamp.toSec())
            {
                std::cout << "no imu data" << std::endl;
                continue;
            }

            // ROS_INFO("get image RGB");
            // this->mBufMutexRGB.lock();
            // imRGB = GetImage(imgRGBBuf.front());
            // imgRGBBuf.pop();
            // this->mBufMutexRGB.unlock();

            // ROS_INFO("get image D");
            // this->mBufMutexD.lock();
            // imD = GetImage(imgDBuf.front());
            // imgDBuf.pop();
            // this->mBufMutexD.unlock();

            ROS_INFO("get image RGBD");
            this->mBufMutexRGB.lock();
            this->mBufMutexD.lock();
            auto [imRGB, imD] = GetImageRGBD(imgRGBBuf.front(), imgDBuf.front());
            imgRGBBuf.pop();
            imgDBuf.pop();
            this->mBufMutexD.unlock();
            this->mBufMutexRGB.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            
            if(!mpImuGb->imuBuf.empty())
            {
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRGB)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // if(mbClahe)
            // {
            //     mClahe->apply(imRGB,imRGB);
            //     mClahe->apply(imD,imD);
            // }

            // Main algorithm runs here
            cv::Mat Tcw_prev = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(imRGB,imD,tImRGB,vImuMeas).matrix());
            ros::Time previous_frame_time = ros::Time::now();
            cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackRGBD(imRGB,imD,tImRGB,vImuMeas).matrix());
            ros::Time current_frame_time = ros::Time::now();

            publish_ros_pose_tf(Tcw, Tcw_prev, previous_frame_time, current_frame_time, ORB_SLAM3::System::IMU_RGBD, max_lin_vel_, max_ang_vel_);

            publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

            // publish_ros_tracking_img(mpSLAM->GetCurrentFrame(), current_frame_time);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}