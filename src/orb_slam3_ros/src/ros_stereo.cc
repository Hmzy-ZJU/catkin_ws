/**
 * @file ros_stereo.cc
 * @brief ORB-SLAM3 ROS stereo node with optional underwater enhancement.
 */

#include <chrono>

#include "System.h"
#include "common.h"
#include "orb_slam3_ros/uw_fusion.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace std;

static uwfusion::Params gUW;
static bool gUW_loaded = false;
static int frame_count = 0;

static void LoadUWParamsFromYAML(const std::string& settings_file)
{
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        return;

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

    gUW_loaded = true;
    ROS_INFO_STREAM("[ros_stereo] UW Fusion: enable=" << (gUW.enable ? "true" : "false")
                    << ", levels=" << gUW.levels
                    << ", scb=" << gUW.scb_percent);
}

class ImageGrabber
{
public:
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                    const sensor_msgs::ImageConstPtr& msgRight);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string node_name = ros::this_node::getName();
    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file;
    std::string settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if(voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file");
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin = false;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, false);

    ROS_INFO_STREAM("[ros_stereo] settings_file: " << settings_file);

    if(!gUW_loaded)
        LoadUWParamsFromYAML(settings_file);

    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(
        node_handler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(
        node_handler, "/camera/right/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    pSLAM->Shutdown();
    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                              const sensor_msgs::ImageConstPtr& msgRight)
{
    ++frame_count;
    const ros::Time msg_time = msgLeft->header.stamp;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(cv_ptrLeft->image.empty() || cv_ptrRight->image.empty())
        return;

    cv::Mat imLeft = cv_ptrLeft->image.clone();
    cv::Mat imRight = cv_ptrRight->image.clone();

    if(gUW.enable)
    {
        cv::Mat bgrLeft;
        cv::Mat bgrRight;
        if(imLeft.channels() == 1)
            cv::cvtColor(imLeft, bgrLeft, cv::COLOR_GRAY2BGR);
        else
            bgrLeft = imLeft;

        if(imRight.channels() == 1)
            cv::cvtColor(imRight, bgrRight, cv::COLOR_GRAY2BGR);
        else
            bgrRight = imRight;

        cv::Mat imLeftEnh;
        cv::Mat imRightEnh;
        auto t_start = std::chrono::steady_clock::now();
        uwfusion::EnhanceFusionBGR(bgrLeft, imLeftEnh, gUW);
        uwfusion::EnhanceFusionBGR(bgrRight, imRightEnh, gUW);
        auto t_end = std::chrono::steady_clock::now();

        const double enh_ms = std::chrono::duration_cast<
            std::chrono::duration<double, std::milli>
        >(t_end - t_start).count();

        if(pSLAM)
            pSLAM->RegisterEnhanceTime(enh_ms);

        imLeft = imLeftEnh;
        imRight = imRightEnh;
    }

    cv::Mat imLeftGray;
    cv::Mat imRightGray;
    if(imLeft.channels() == 3)
        cv::cvtColor(imLeft, imLeftGray, cv::COLOR_BGR2GRAY);
    else
        imLeftGray = imLeft;

    if(imRight.channels() == 3)
        cv::cvtColor(imRight, imRightGray, cv::COLOR_BGR2GRAY);
    else
        imRightGray = imRight;

    try
    {
        pSLAM->TrackStereo(imLeftGray, imRightGray, msg_time.toSec());
    }
    catch(const cv::Exception& e)
    {
        ROS_WARN_THROTTLE(5.0,
            "[ros_stereo] TrackStereo OpenCV exception: %s left=%dx%d right=%dx%d stamp=%.9f",
            e.what(),
            imLeftGray.cols, imLeftGray.rows,
            imRightGray.cols, imRightGray.rows,
            msg_time.toSec());
        return;
    }
    catch(const std::exception& e)
    {
        ROS_WARN_THROTTLE(5.0, "[ros_stereo] TrackStereo exception: %s", e.what());
        return;
    }

    publish_topics(msg_time);

    if(frame_count == 1 || frame_count % 100 == 0)
        ROS_INFO("[ros_stereo] Processed %d frames", frame_count);
}
