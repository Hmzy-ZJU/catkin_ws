/** 
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
*
*/

#include "common.h"
#include "orb_slam3_ros/uw_fusion.h"   // <<< 新增：水下图像增强（Ancuti 融合）
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

// ----------------------- UW Fusion 参数加载 -----------------------
static uwfusion::Params gUW;
static bool gUW_loaded = false;

static void LoadUWParams(ros::NodeHandle& nhp) {
    // 私有命名空间（~uw_fusion/*）
    nhp.param("uw_fusion/enable",                gUW.enable,                false);
    nhp.param("uw_fusion/levels",                gUW.levels,                5);
    nhp.param("uw_fusion/scb_percent",           gUW.scb_percent,           1.0);
    nhp.param("uw_fusion/bilateral_d",           gUW.bilateral_d,           5);
    nhp.param("uw_fusion/bilateral_sigma_color", gUW.bilateral_sigma_color, 5.0);
    nhp.param("uw_fusion/bilateral_sigma_space", gUW.bilateral_sigma_space, 5.0);
    nhp.param("uw_fusion/clahe_clip",            gUW.clahe_clip,            2.0);
    nhp.param("uw_fusion/clahe_tile",            gUW.clahe_tile,            8);
    nhp.param("uw_fusion/exposed_avg",           gUW.exposed_avg,           0.5);
    nhp.param("uw_fusion/exposed_sigma",         gUW.exposed_sigma,         0.25);
    gUW_loaded = true;

    ROS_INFO_STREAM("[ros_rgbd] UW Fusion params: enable=" << (gUW.enable?"true":"false")
                    << ", levels=" << gUW.levels
                    << ", scb=" << gUW.scb_percent
                    << ", bilateral(d=" << gUW.bilateral_d
                    << ", sigC=" << gUW.bilateral_sigma_color
                    << ", sigS=" << gUW.bilateral_sigma_space << ")"
                    << ", clahe(clip=" << gUW.clahe_clip
                    << ", tile=" << gUW.clahe_tile << ")"
                    << ", exposed(avg=" << gUW.exposed_avg
                    << ", sigma=" << gUW.exposed_sigma << ")");
}

// ----------------------- 将ROS彩色图像安全转为BGR8 -----------------------
static bool ToBGR8(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bgr) {
    try {
        const std::string& enc = msg->encoding;
        if (enc == sensor_msgs::image_encodings::BGR8) {
            bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
            return true;
        } else if (enc == sensor_msgs::image_encodings::RGB8) {
            cv::Mat rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            return true;
        } else if (enc == sensor_msgs::image_encodings::MONO8) {
            cv::Mat gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return true;
        } else {
            // 其他编码尝试直接请求 BGR8（若驱动支持）
            bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
            return true;
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in ToBGR8: %s", e.what());
        return false;
    }
}

// ----------------------- 图像抓取类 -----------------------
class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;      // 公有句柄
    ros::NodeHandle private_nh("~");   // 私有句柄（用于 uw_fusion 参数）
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

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // 读取 UW Fusion 参数（一次即可）
    if (!gUW_loaded) LoadUWParams(private_nh);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // RGB: 转为 BGR8
    cv::Mat rgb_bgr;
    if (!ToBGR8(msgRGB, rgb_bgr)) {
        ROS_ERROR("Failed to convert RGB image to BGR8.");
        return;
    }

    // 深度：按原编码直接共享（不做任何处理）
    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD); // 通常为 16UC1 或 32FC1
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception (Depth): %s", e.what());
        return;
    }

    // —— 水下图像增强（Ancuti 融合）：仅对彩色图像；由开关控制
    cv::Mat rgb_enh_bgr;
    uwfusion::EnhanceFusionBGR(rgb_bgr, rgb_enh_bgr, gUW); // 若 gUW.enable=false，将原样回传

    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f Tcw = pSLAM->TrackRGBD(rgb_enh_bgr, cv_ptrD->image, msgRGB->header.stamp.toSec());

    ros::Time msg_time = msgRGB->header.stamp;
    publish_topics(msg_time);
}
