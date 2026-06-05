/**
*
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include <chrono>
#include "common.h"
#include "orb_slam3_ros/uw_fusion.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <atomic>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

static uwfusion::Params gUW;
static bool gUW_loaded = false;
static double gCamImuTimeShift = 0.0;

static void LoadUWParamsFromYAML(const std::string& settings_file)
{
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_WARN_STREAM("[ros_mono_inertial] Cannot open settings file " << settings_file
                        << " for UW-Fusion params; using defaults.");
        return;
    }

    cv::FileNode n = fs["uwfusion.enable"];
    if(!n.empty()) gUW.enable = ((int)n != 0);
    n = fs["uwfusion.levels"]; if(!n.empty()) gUW.levels = (int)n;
    n = fs["uwfusion.scb_percent"]; if(!n.empty()) gUW.scb_percent = (double)n;
    n = fs["uwfusion.bilateral_d"]; if(!n.empty()) gUW.bilateral_d = (int)n;
    n = fs["uwfusion.bilateral_sigma_color"]; if(!n.empty()) gUW.bilateral_sigma_color = (double)n;
    n = fs["uwfusion.bilateral_sigma_space"]; if(!n.empty()) gUW.bilateral_sigma_space = (double)n;
    n = fs["uwfusion.clahe_clip"]; if(!n.empty()) gUW.clahe_clip = (double)n;
    n = fs["uwfusion.clahe_tile"]; if(!n.empty()) gUW.clahe_tile = (int)n;
    n = fs["uwfusion.exposed_avg"]; if(!n.empty()) gUW.exposed_avg = (double)n;
    n = fs["uwfusion.exposed_sigma"]; if(!n.empty()) gUW.exposed_sigma = (double)n;

    n = fs["timeshift_cam_imu"];
    if(!n.empty()) gCamImuTimeShift = (double)n;

    gUW_loaded = true;

    ROS_INFO_STREAM("[ros_mono_inertial] UW Fusion: enable=" << (gUW.enable ? "true" : "false")
                    << ", levels=" << gUW.levels
                    << ", scb=" << gUW.scb_percent);
    ROS_INFO_STREAM("[ros_mono_inertial] Camera-IMU timeshift: "
                    << std::showpos << gCamImuTimeShift << " s"
                    << std::noshowpos << " (t_sync = t_cam + timeshift_cam_imu)");
}

static bool ToBGR8(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bgr)
{
    try
    {
        const std::string& enc = msg->encoding;
        if (enc == sensor_msgs::image_encodings::BGR8)
        {
            bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
            return true;
        }
        else if (enc == sensor_msgs::image_encodings::RGB8)
        {
            cv::Mat rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            return true;
        }
        else if (enc == sensor_msgs::image_encodings::MONO8)
        {
            cv::Mat gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return true;
        }

        bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        return true;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in ToBGR8: %s", e.what());
        return false;
    }
}

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
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb), mbShutdown(false){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void RequestShutdown();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
    std::atomic<bool> mbShutdown;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    if (!gUW_loaded)
        LoadUWParamsFromYAML(settings_file);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);
    
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    igb.RequestShutdown();
    if (sync_thread.joinable())
        sync_thread.join();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    img0Buf.push(img_msg);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv::Mat bgr;
    if (!ToBGR8(img_msg, bgr))
        return cv::Mat();

    cv::Mat bgr_enh;
    auto t_enh_start = std::chrono::steady_clock::now();
    uwfusion::EnhanceFusionBGR(bgr, bgr_enh, gUW);
    auto t_enh_end = std::chrono::steady_clock::now();

    if (pSLAM && gUW.enable)
    {
        const double enh_ms = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>
        >(t_enh_end - t_enh_start).count();
        pSLAM->RegisterEnhanceTime(enh_ms);
    }

    cv::Mat gray;
    cv::cvtColor(bgr_enh, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

void ImageGrabber::SyncWithImu()
{
    while (!mbShutdown.load())
    {
        sensor_msgs::ImageConstPtr front_img;
        double latest_imu_time = -1.0;
        {
            std::lock_guard<std::mutex> img_lock(this->mBufMutex);
            std::lock_guard<std::mutex> imu_lock(mpImuGb->mBufMutex);
            if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
            {
                front_img = img0Buf.front();
                latest_imu_time = mpImuGb->imuBuf.back()->header.stamp.toSec();
            }
        }

        if (!front_img)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const double tImgRaw = front_img->header.stamp.toSec();
        const double tIm = tImgRaw + gCamImuTimeShift;

        if (tIm < 0.0)
        {
            std::lock_guard<std::mutex> lock(this->mBufMutex);
            if (!img0Buf.empty())
            {
                img0Buf.pop();
            }
            ROS_WARN_THROTTLE(5.0, "[ros_mono_inertial] Corrected image timestamp is negative. Dropping frame.");
            continue;
        }

        if (tIm > latest_imu_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        sensor_msgs::ImageConstPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(this->mBufMutex);
            if (img0Buf.empty())
            {
                continue;
            }
            img_msg = img0Buf.front();
            img0Buf.pop();
        }

        cv::Mat im = GetImage(img_msg);
        if (im.empty())
        {
            continue;
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        Eigen::Vector3f Wbb = Eigen::Vector3f::Zero();
        {
            std::lock_guard<std::mutex> imu_lock(mpImuGb->mBufMutex);
            while (!mpImuGb->imuBuf.empty() &&
                   mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
            {
                const sensor_msgs::ImuConstPtr& imu_msg = mpImuGb->imuBuf.front();
                const double t = imu_msg->header.stamp.toSec();

                cv::Point3f acc(
                    imu_msg->linear_acceleration.x,
                    imu_msg->linear_acceleration.y,
                    imu_msg->linear_acceleration.z);

                cv::Point3f gyr(
                    imu_msg->angular_velocity.x,
                    imu_msg->angular_velocity.y,
                    imu_msg->angular_velocity.z);

                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                Wbb << imu_msg->angular_velocity.x,
                       imu_msg->angular_velocity.y,
                       imu_msg->angular_velocity.z;

                mpImuGb->imuBuf.pop();
            }
        }

        pSLAM->TrackMonocular(im, tIm, vImuMeas);
        publish_topics(img_msg->header.stamp, Wbb);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ImageGrabber::RequestShutdown()
{
    mbShutdown.store(true);
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);

    return;
}
                                                                                                                                                                  