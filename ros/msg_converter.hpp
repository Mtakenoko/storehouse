#ifndef HTL_MSG_CONVERTER__
#define HTL_MSG_CONVERTER__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace htl
{
    class Converter
    {
    public:
        static void cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg);
        static void msg_to_cvimage(const sensor_msgs::msg::Image::SharedPtr &msg, cv::Mat &frame);
        static std::string cvMattype_to_Encoding(int mat_type);
        static int Encoding_to_cvMattype(const std::string &encoding);
    };
} // namespace htl

std::string htl::Converter::cvMattype_to_Encoding(int mat_type)
{
    switch (mat_type)
    {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

int htl::Converter::Encoding_to_cvMattype(const std::string &encoding)
{
    if (encoding == "mono8")
    {
        return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
        return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
        return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
        return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
        return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
        return CV_8UC3;
    }
    else
    {
        throw std::runtime_error("Unsupported encoding type");
    }
}

void htl::Converter::cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
{
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = htl::Converter::cvMattype_to_Encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}

void htl::Converter::msg_to_cvimage(const sensor_msgs::msg::Image::SharedPtr &msg_image, cv::Mat &frame)
{
    cv::Mat frame_image(msg_image->height, msg_image->width, htl::Converter::Encoding_to_cvMattype(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    if (msg_image->encoding == "rgb8")
    {
        cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
    }
    frame = frame_image.clone();
}

#endif