/**
*
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/
#include <chrono>
#include "System.h"

#include "common.h"
#include "orb_slam3_ros/uw_fusion.h"   // <<< 新增：水下融合头文件
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

using namespace std;

// ----------------------- UW Fusion 参数加载 -----------------------
static uwfusion::Params gUW;
static bool gUW_loaded = false;
// 从 ORB-SLAM3 的 settings.yaml 中读取 uwfusion.* 参数
static void LoadUWParamsFromYAML(const std::string& settings_file)
{
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_WARN_STREAM("[ros_mono] Cannot open settings file " << settings_file
                        << " for UW-Fusion params; using defaults in uw_fusion::Params.");
        return;
    }

    // 注意：gUW 在 uw_fusion.h 里有一套默认值
    // 我们只对 YAML 里出现的字段做覆盖，没有的就保持默认

    // enable（允许 0/1 或 true/false -> 用 int 强制转）
    {
        cv::FileNode n = fs["uwfusion.enable"];
        if(!n.empty())
        {
            int v = (int)n;
            gUW.enable = (v != 0);
        }
    }

    cv::FileNode n;

    n = fs["uwfusion.levels"];
    if(!n.empty()) gUW.levels = (int)n;

    n = fs["uwfusion.scb_percent"];
    if(!n.empty()) gUW.scb_percent = (double)n;

    n = fs["uwfusion.bilateral_d"];
    if(!n.empty()) gUW.bilateral_d = (int)n;

    n = fs["uwfusion.bilateral_sigma_color"];
    if(!n.empty()) gUW.bilateral_sigma_color = (double)n;

    n = fs["uwfusion.bilateral_sigma_space"];
    if(!n.empty()) gUW.bilateral_sigma_space = (double)n;

    n = fs["uwfusion.clahe_clip"];
    if(!n.empty()) gUW.clahe_clip = (double)n;

    n = fs["uwfusion.clahe_tile"];
    if(!n.empty()) gUW.clahe_tile = (int)n;

    n = fs["uwfusion.exposed_avg"];
    if(!n.empty()) gUW.exposed_avg = (double)n;

    n = fs["uwfusion.exposed_sigma"];
    if(!n.empty()) gUW.exposed_sigma = (double)n;

    gUW_loaded = true;

    ROS_INFO_STREAM("[ros_mono] UW Fusion params (from YAML): enable=" << (gUW.enable?"true":"false")
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

// ----------------------- BGR8 安全获取工具 -----------------------
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
            // 尝试直接转 bgr8（若驱动支持）
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

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;      // 公有句柄
    ros::NodeHandle private_nh("~");   // 私有句柄（给其它私有参数用，uwfusion 不再走 ROS）
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
    // 打印一下真正拿到的 settings_file 是哪一个
    ROS_INFO_STREAM("[ros_mono] settings_file param = " << settings_file);
    // === 从 YAML 读取 UW-Fusion 参数（像 Info 一样） ===
    if (!gUW_loaded)
        LoadUWParamsFromYAML(settings_file);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb;

    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

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
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) 
{
    // 将 ROS Image 转成 BGR8（统一入口）
    cv::Mat bgr;
    if (!ToBGR8(msg, bgr)) {
        ROS_ERROR("Failed to convert input image to BGR8.");
        return;
    }

    // —— 水下图像增强（Ancuti 融合）：由开关控制
    cv::Mat bgr_enh;

    // 计时开始：只统计增强模块的耗时
    auto t_enh_start = std::chrono::steady_clock::now();
    uwfusion::EnhanceFusionBGR(bgr, bgr_enh, gUW);   // 若 gUW.enable = false，则内部直接返回原图
    auto t_enh_end   = std::chrono::steady_clock::now();

    // 计算毫秒
    double enh_ms = std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>
                    >(t_enh_end - t_enh_start).count();

    // 只有在增强开关打开时，才把这次耗时记到系统统计里
    if (pSLAM && gUW.enable)
    {
        // 这个函数是你在 ORB_SLAM3::System 里加的统计接口
        // 内部通常是转发给 Tracking::mRtStats.enh_ms.push_back(...)
        pSLAM->RegisterEnhanceTime(enh_ms);
    }

    // ORB-SLAM3 runs in TrackMonocular()
    // 注意：TrackMonocular 可接受 8UC1/8UC3；内部会自行转灰度
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(bgr_enh, msg->header.stamp.toSec());

    ros::Time msg_time = msg->header.stamp;
    publish_topics(msg_time);
}
