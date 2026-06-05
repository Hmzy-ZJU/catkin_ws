/**
 * @file ros_stereo_minimal.cc
 * @brief 最小化双目测试 - 绕过所有可能的问题点
 */

#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>

// ORB-SLAM3 - 只包含必要的头文件
#include "System.h"

using namespace std;

ORB_SLAM3::System* pSLAM = nullptr;
int frame_count = 0;

class ImageGrabber
{
public:
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, 
                    const sensor_msgs::ImageConstPtr& msgRight);
};

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                              const sensor_msgs::ImageConstPtr& msgRight)
{
    frame_count++;
    
    ROS_INFO_STREAM_THROTTLE(1.0, "[MINIMAL] Frame " << frame_count 
                        << " | Left: " << msgLeft->width << "x" << msgLeft->height
                        << " | Right: " << msgRight->width << "x" << msgRight->height);

    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptrLeft->image.empty() || cv_ptrRight->image.empty()) {
        ROS_ERROR("Empty image!");
        return;
    }

    double timestamp = msgLeft->header.stamp.toSec();

    try {
        Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, timestamp);
        
        // 只打印位置，不做任何其他处理
        if (!Tcw.translation().array().isNaN()[0]) {
            ROS_INFO_STREAM_THROTTLE(1.0, "[MINIMAL] Position: " 
                << Tcw.translation().x() << ", "
                << Tcw.translation().y() << ", "
                << Tcw.translation().z());
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("[MINIMAL] TrackStereo exception: " << e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_minimal");
    ros::NodeHandle nh;

    std::string node_name = ros::this_node::getName();

    std::string voc_file, settings_file;
    nh.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    nh.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set") {
        ROS_ERROR("Please provide voc_file and settings_file");
        return 1;
    }

    ROS_INFO_STREAM("[MINIMAL] voc_file: " << voc_file);
    ROS_INFO_STREAM("[MINIMAL] settings_file: " << settings_file);
    ROS_INFO("[MINIMAL] Creating ORB-SLAM3 system (NO visualization)...");
    
    try {
        // 关键：第4个参数 false 禁用可视化
        pSLAM = new ORB_SLAM3::System(voc_file, settings_file, ORB_SLAM3::System::STEREO, false);
        ROS_INFO("[MINIMAL] ORB-SLAM3 system created successfully!");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("[MINIMAL] Failed to create SLAM system: " << e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR("[MINIMAL] Unknown exception creating SLAM system!");
        return 1;
    }

    ImageGrabber igb;
    
    message_filters::Subscriber<sensor_msgs::Image> sub_left(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_right(nh, "/camera/right/image_raw", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_left, sub_right);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    ROS_INFO("[MINIMAL] Waiting for stereo images...");

    ros::spin();

    ROS_INFO("[MINIMAL] Shutting down...");
    pSLAM->Shutdown();

    return 0;
}
