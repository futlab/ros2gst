#include <ros/ros.h>
#include <gst/app/gstappsrc.h>
#include <gst/pbutils/missing-plugins.h>
#include "pipeline.h"

Pipeline::Pipeline(const cv::Size &size, bool color, const std::string &pipelineDesc, double fps) :
    pipeline_(nullptr), source_(nullptr), color_(color), imageSize_(size), fps_(fps), frameCount_(0), buffer_(nullptr)
{
    gst_init (NULL, NULL);
    GError *err = NULL;
    pipeline_ = gst_parse_launch(pipelineDesc.c_str(), &err);
    if (err)
        ROS_ERROR("gst_parse_launch() error: %s", err->message);
    if (!pipeline_)
        ROS_FATAL("gst_parse_launch() returned NULL");
    GstIterator *it = gst_bin_iterate_sources(GST_BIN(pipeline_));
    GValue value = G_VALUE_INIT;
    GstIteratorResult r;
    do {
        r = gst_iterator_next(it, &value);
        if (r == GST_ITERATOR_OK) {
            GstElement *element = GST_ELEMENT (g_value_get_object (&value));
            if(gchar* name = gst_element_get_name(element)) {
                if(strstr(name, "appsrc"))
                    source_ = GST_ELEMENT (gst_object_ref(element));
                g_free(name);
            }

        } else
            if (r == GST_ITERATOR_RESYNC)
                gst_iterator_resync (it);
    } while (!source_ && r != GST_ITERATOR_DONE && r != GST_ITERATOR_ERROR);
    gst_iterator_free(it);
    if (!source_)
        ROS_FATAL("GStreamer: cannot find appsrc in pipeline");

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, color ? "BGR" : "GRAY8",
                               "width", G_TYPE_INT, size.width,
                               "height", G_TYPE_INT, size.height,
                               "framerate", GST_TYPE_FRACTION, int(fps), 1,
                               NULL);
    caps = gst_caps_fixate(caps);
    gst_app_src_set_caps(GST_APP_SRC(source_), caps);
    gst_app_src_set_stream_type(GST_APP_SRC(source_), GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_size (GST_APP_SRC(source_), -1);
    g_object_set(G_OBJECT(source_), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(source_), "block", 1, NULL);
    g_object_set(G_OBJECT(source_), "is-live", 0, NULL);

    if(gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        handleMessages();
        ROS_FATAL("gst_element_set_state(): cannot put pipeline to play");
    }
    handleMessages();
}

void Pipeline::write(const cv::Mat &image)
{
    handleMessages();
    assert((color_ && image.type() == CV_8UC3) || (!color_ && image.type() == CV_8U));
    if (image.cols != imageSize_.width || image.rows != imageSize_.height)
        ROS_FATAL("Size %d x %d changed to %d x %d", imageSize_.width, imageSize_.height, image.cols, image.rows);

    size_t size = image.dataend - image.datastart;
    buffer_ = gst_buffer_new_allocate (NULL, size, NULL);
    GstMapInfo info;
    gst_buffer_map(buffer_, &info, (GstMapFlags)GST_MAP_READ);
    memcpy(info.data, image.datastart, size);
    gst_buffer_unmap(buffer_, &info);

    GstClockTime duration = ((double)1/fps_) * GST_SECOND, timestamp = frameCount_ * duration;
    GST_BUFFER_DURATION(buffer_) = duration;
    GST_BUFFER_PTS(buffer_) = timestamp;
    GST_BUFFER_DTS(buffer_) = timestamp;
    GST_BUFFER_OFFSET(buffer_) =  frameCount_;

    if (gst_app_src_push_buffer(GST_APP_SRC(source_), buffer_) != GST_FLOW_OK) {
        ROS_WARN("gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
        return;
    }
    //GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
    ++frameCount_;
}

Pipeline::~Pipeline()
{
    if (pipeline_)
    {
        handleMessages();

        if (gst_app_src_end_of_stream(GST_APP_SRC(source_)) != GST_FLOW_OK)
        {
            ROS_WARN("Cannot send EOS to GStreamer pipeline\n");
            return;
        }

        //wait for EOS to trickle down the pipeline. This will let all elements finish properly
        GstBus* bus = gst_element_get_bus(pipeline_);
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
        {
            ROS_WARN("gst_bus_timed_pop_filtered(): Error during finalization");
            return;
        }

        if(msg != NULL)
        {
            gst_message_unref(msg);
            g_object_unref(G_OBJECT(bus));
        }

        GstStateChangeReturn status = gst_element_set_state (pipeline_, GST_STATE_NULL);
        if (status == GST_STATE_CHANGE_ASYNC)
        {
            // wait for status update
            GstState st1;
            GstState st2;
            status = gst_element_get_state(pipeline_, &st1, &st2, GST_CLOCK_TIME_NONE);
        }
        if (status == GST_STATE_CHANGE_FAILURE)
        {
            handleMessages();
            gst_object_unref (GST_OBJECT (pipeline_));
            ROS_WARN("Unable to stop gstreamer pipeline");
            return;
        }
        gst_object_unref (GST_OBJECT (pipeline_));
    }
}

void Pipeline::handleMessages()
{
    GError *err = NULL;
    gchar *debug = NULL;
    GstBus *bus = gst_element_get_bus(pipeline_);
    while(gst_bus_have_pending(bus)) {
        GstMessage *msg = gst_bus_pop(bus);
        if(gst_is_missing_plugin_message(msg)) {
            ROS_ERROR("gst_is_missing_plugin_message(): your gstreamer installation is missing a required plugin");
        } else {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_STATE_CHANGED:
                GstState oldstate, newstate, pendstate;
                gst_message_parse_state_changed(msg, &oldstate, &newstate, &pendstate);
                /*ROS_INFO("%s: state changed from %s to %s (pending: %s)",
                    gst_element_get_name(GST_MESSAGE_SRC (msg)),
                    gst_element_state_get_name(oldstate),
                    gst_element_state_get_name(newstate), gst_element_state_get_name(pendstate));*/
                break;
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug);
                ROS_ERROR("GStreamer Plugin: Embedded video playback halted; module %s reported: %s",
                    gst_element_get_name(GST_MESSAGE_SRC (msg)), err->message);
                g_error_free(err);
                g_free(debug);
                gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_NULL);
                break;
            case GST_MESSAGE_STREAM_STATUS:
                GstStreamStatusType tp;
                GstElement *elem;
                gst_message_parse_stream_status(msg, &tp, &elem);
                //ROS_INFO("stream status: elem %s, %i", GST_ELEMENT_NAME(elem), tp);
                break;
            default:
                //ROS_WARN("unhandled message %s",GST_MESSAGE_TYPE_NAME(msg));
                break;
            }
            gst_message_unref(msg);
        }
    }
    gst_object_unref(GST_OBJECT(bus));
}

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

std::string buildPipelineDesc(const std::string &udpTarget, cv::Size size, int fps, bool color)
{
    using namespace std;
    int port = 5600;
    string addr;
    parseTarget(udpTarget, &addr, &port);
    string src = "appsrc ! video/x-raw,format=" + string(color ? "BGR" : "GRAY8") + ",width=" + to_string(size.width) + ",height=" + to_string(size.height) + ",framerate=" + to_string(fps) + "/1 ";
    string conv = "! videoconvert ! video/x-raw,format=I420 ";
    string enc = "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast intra-refresh=true key-int-max=3 ! rtph264pay ";
    string sink = "! udpsink host=" + addr + " port=" + to_string(port);
    return src + conv + enc + sink;
}
