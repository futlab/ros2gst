
// example appsrc for gstreamer 1.0 with own mainloop & external buffers. based on example from gstreamer docs.
// public domain, 2015 by Florian Echtler <floe@butterbrot.org>. compile with:
// gcc --std=c99 -Wall $(pkg-config --cflags gstreamer-1.0) -o gst gst.c $(pkg-config --libs gstreamer-1.0) -lgstapp-1.0

#include <stdio.h>
#include <stdint.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include "pipeline.h"

std::unique_ptr<Pipeline> pipeline;
std::string udpTarget;
double fps = 15;

void parseTarget(const std::string &target, std::string *addr, int *port)
{
    auto pos = target.find(':');
    if (pos == std::string::npos) {
        *addr = target;
    } else {
        *addr = target.substr(0, pos);
        *port = std::atoi(target.c_str() + pos + 1);
    }
}

void makePipeline(int w, int h, bool c)
{
    using namespace std;
    string addr;
    int port = 5600;
    parseTarget(udpTarget, &addr, &port);
    string src = "appsrc ! video/x-raw,format=" + string(c ? "BGR" : "GRAY8") + ",width=" + to_string(w) + ",height=" + to_string(h) + ",framerate=" + to_string((int)fps) + "/1 ";
    string conv = "! videoconvert ! video/x-raw,format=I420 ";
    string enc = "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ";
    string sink = "! udpsink host=" + addr + " port=" + to_string(port);
    string full = src + conv + enc + sink;
    ROS_INFO("cmd: gst-launch-1.0 -v %s", full.c_str());
    pipeline = make_unique<Pipeline>(cv::Size(w, h), c, full, fps);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO_ONCE("Image received");
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!pipeline)
        makePipeline(image.cols, image.rows, true);
    pipeline->write(image);
}

void disparityCallback(const stereo_msgs::DisparityImage &msg)
{
    ROS_INFO_ONCE("Disparity received");
    try
    {
        auto &mi = msg.image;
        cv::Mat fi( mi.height, mi.width, CV_32FC1, (void *)&(mi.data[0])), bi;
        fi.convertTo(bi, CV_8U, 255.0 / msg.max_disparity);
        if(!pipeline) {
            makePipeline(bi.cols, bi.rows, false);
        }
        pipeline->write(bi);
    }
    catch (cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

gint main(gint argc, gchar *argv[])
{
    ROS_INFO_ONCE("Waiting for images");

    ros::init(argc, argv, "ros2gst");
    ros::NodeHandle nh, nhp("~");
    if (!nhp.getParam("udp_target", udpTarget)) {
        ROS_FATAL("Unable to get udp_target");
    }

    ros::Subscriber subImg = nh.subscribe("image", 5, &imageCallback);
    ros::Subscriber subDisparity = nh.subscribe("disparity", 5, &disparityCallback);

    ros::spin();
    pipeline.reset();
    return 0;
}
