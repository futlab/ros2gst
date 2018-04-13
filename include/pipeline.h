#ifndef PIPELINE_H
#define PIPELINE_H
#include <string>
#include <gst/gst.h>
#include <opencv2/core/core.hpp>

class Pipeline
{
    GstElement *pipeline_, *source_;
    const bool color_;
    const cv::Size imageSize_;
    const double fps_;
    int frameCount_;
    GstBuffer* buffer_;
    void handleMessages();
public:
    Pipeline(const cv::Size &size, bool color, const std::string &pipelineDesc, double fps = 15);
    void write(const cv::Mat &image);
    ~Pipeline();
};

#endif // PIPELINE_H
