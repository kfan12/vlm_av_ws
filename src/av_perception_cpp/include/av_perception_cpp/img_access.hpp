#pragma once
// Zero-copy sensor_msgs/Image -> cv::Mat. NO cv_bridge anywhere in v2 (the
// venv NumPy-2 issue bans the Python binding; C++ simply doesn't need it).
// The Mat aliases the message buffer: keep the ConstSharedPtr alive while
// using the Mat.
#include <stdexcept>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av::img
{

    inline cv::Mat as_rgb(const sensor_msgs::msg::Image::ConstSharedPtr &m)
    {
        if (m->encoding != "rgb8")
            throw std::runtime_error("img_access: expected rgb8, got " + m->encoding);
        return cv::Mat(m->height, m->width, CV_8UC3,
                       const_cast<uint8_t *>(m->data.data()), m->step);
    }

    inline cv::Mat as_depth(const sensor_msgs::msg::Image::ConstSharedPtr &m)
    {
        if (m->encoding != "32FC1")
            throw std::runtime_error("img_access: expected 32FC1, got " + m->encoding);
        return cv::Mat(m->height, m->width, CV_32FC1,
                       const_cast<uint8_t *>(m->data.data()), m->step);
    }

} // namespace av::img
