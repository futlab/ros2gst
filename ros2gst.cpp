
// example appsrc for gstreamer 1.0 with own mainloop & external buffers. based on example from gstreamer docs.
// public domain, 2015 by Florian Echtler <floe@butterbrot.org>. compile with:
// gcc --std=c99 -Wall $(pkg-config --cflags gstreamer-1.0) -o gst gst.c $(pkg-config --libs gstreamer-1.0) -lgstapp-1.0

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>

#include <stdint.h>

static void prepare_buffer(GstAppSrc* appsrc, cv::Mat image) {

  static GstClockTime timestamp = 0;
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  //if (!want) return;
  //want = 0;

  void *data = (void *)image.datastart;
  size = image.dataend - image.datastart;

  buffer = gst_buffer_new_wrapped_full( GST_MEMORY_FLAG_NO_SHARE, (gpointer)data, size, 0, size, NULL, NULL );

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 4);

  timestamp += GST_BUFFER_DURATION (buffer);

  ret = gst_app_src_push_buffer(appsrc, buffer);

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    // g_main_loop_quit (loop);
  }
}

GstElement *pipeline = nullptr;
std::string udpTarget;

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

GstElement *makePipeline(int width, int height, const std::string &udpTarget, bool color = true)
{
    GstElement *appsrc, *conv, *videosink;

    // init GStreamer
    gst_init (NULL, NULL);

    // setup pipeline
    pipeline = gst_pipeline_new ("pipeline");
    appsrc = gst_element_factory_make ("appsrc", "source");
    GstElement *test = gst_element_factory_make ("videotestsrc", "test");
    GstElement *scale = gst_element_factory_make ("videoscale", "videoscale");
    GstElement *filter  = gst_element_factory_make("capsfilter", "caps_filter1");
    conv = gst_element_factory_make ("videoconvert", "conv");
    videosink = gst_element_factory_make ("xvimagesink", "videosink");
    GstElement *enc = gst_element_factory_make("x264enc", "enc");
    GstElement *pay = gst_element_factory_make("rtph264pay", "pay");
    GstElement *udp = gst_element_factory_make("udpsink", "udpsink");

    // setup

    gst_util_set_object_arg(G_OBJECT (test), "pattern", "colors");

    g_object_set (G_OBJECT (appsrc), "caps",
                  gst_caps_new_simple ("video/x-raw",
                                       "format", G_TYPE_STRING, color ? "BGR" : "GRAY8",
                                       "width", G_TYPE_INT, width,
                                       "height", G_TYPE_INT, height,
                                       "framerate", GST_TYPE_FRACTION, 30, 1,
                                       NULL), NULL);

    g_object_set (G_OBJECT (filter), "caps",
                  gst_caps_new_simple ("video/x-raw",
                                       "format", G_TYPE_STRING, "I420", //<<< IF THIS IS SET TO ARGB (THE FORMAT I WANT IT FAILS ON LINKING)
                                       "framerate", GST_TYPE_FRACTION, 30, 1,
                                       "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1,
                                       "width", G_TYPE_INT, width,
                                       "height", G_TYPE_INT, height,
                                       NULL), NULL);

    gst_util_set_object_arg(G_OBJECT (enc), "tune", "zerolatency");
    gst_util_set_object_arg(G_OBJECT (enc), "speed-preset", "superfast");
    g_object_set (G_OBJECT (enc), "bitrate", 500, NULL);

    std::string addr;
    int port = 5600;
    parseTarget(udpTarget, &addr, &port);
    g_object_set (G_OBJECT (udp), "host", addr, NULL);
    g_object_set (G_OBJECT (udp), "port", port, NULL);

    gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, filter, enc, pay, udp, NULL);
    if (!gst_element_link_many (appsrc, conv, filter, enc, pay, udp, NULL))
        return nullptr;

    // setup appsrc
    g_object_set (G_OBJECT (appsrc),
                  "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
                  "format", GST_FORMAT_TIME,
                  "is-live", TRUE,
                  NULL);
    //g_signal_connect (appsrc, "need-data", G_CALLBACK (cb_need_data), NULL);

    // play
    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    return appsrc;
}

GstElement *appsrc = nullptr;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO_ONCE("Image received");
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if(!appsrc) {
        appsrc = makePipeline(image.cols, image.rows, udpTarget);
        if (!appsrc) {
            ROS_FATAL("Unable to make pipeline");
            return;
        }
    }

    prepare_buffer((GstAppSrc*)appsrc, image);
}

void disparityCallback(const stereo_msgs::DisparityImage &msg)
{
    ROS_INFO_ONCE("Disparity received");
    try
    {
        auto &mi = msg.image;
        cv::Mat fi( mi.height, mi.width, CV_32FC1, (void *)&(mi.data[0])), bi;
        fi.convertTo(bi, CV_8U, 255.0 / msg.max_disparity);
        //drawHist(bi);
        if(!appsrc) {
            appsrc = makePipeline(bi.cols, bi.rows, udpTarget, false);
            if (!appsrc) {
                ROS_FATAL("Unable to make pipeline");
                return;
            }
        }
        prepare_buffer((GstAppSrc*)appsrc, bi);
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

    // clean up
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline));

    return 0;
}
