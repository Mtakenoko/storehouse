#ifndef MSG_CONVERTER__
#define MSG_CONVERTER__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace htl
{
    class Converter
    {
    public:
        static void cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg);
        static void cvMat_to_msgPointCloud(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, const size_t &color);
        static void cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count = 0);
        static std::string mat_type2encoding(int mat_type);
        static pcl::PointXYZRGB pclColor(const size_t &color);

        enum Color
        {
            WHITE = 0,
            BRACK = 1,
            SILVER = 2,
            BLUE = 3,
            RED = 4,
            GREEN = 5,
            YELLOW = 6,
            AQUA = 7,
            PINK = 8,
            OLIVE = 9,
            TEAL = 10,
            SKIN = 11
        };
    };
} // namespace htl

std::string htl::Converter::mat_type2encoding(int mat_type)
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

void htl::Converter::cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
{
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = Converter::mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}

void htl::Converter::cvMat_to_msgPointCloud(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, const size_t &color)
{

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    for (int i = 0; i < pointCloud2.rows; i++)
    {
        //　色
        pcl::PointXYZRGB pt;
        pt = Converter::pclColor(color);
        pt.x = pointCloud2.at<float>(i, 0);
        pt.y = pointCloud2.at<float>(i, 1);
        pt.z = pointCloud2.at<float>(i, 2);
        cloud_.push_back(pt);
    }
    // auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, msg_cloud_pub);

    // msg_cloud_pub.header = std_msgs::msg::Header();
    // msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";
}

void htl::Converter::cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count)
{
    // msg_cloud_pub.header = std_msgs::msg::Header();
    // msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";

    msg_cloud_pub.is_bigendian = false;
    msg_cloud_pub.is_dense = true;

    msg_cloud_pub.height = 1;
    msg_cloud_pub.width = pointCloud2.rows;

    msg_cloud_pub.fields.resize(3);
    msg_cloud_pub.fields[0].name = "x";
    msg_cloud_pub.fields[1].name = "y";
    msg_cloud_pub.fields[2].name = "z";

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    for (uint32_t i = 0; i < msg_cloud_pub.fields.size(); ++i, offset += sizeof(float))
    {
        msg_cloud_pub.fields[i].count = 1;
        msg_cloud_pub.fields[i].offset = offset;
        msg_cloud_pub.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    msg_cloud_pub.point_step = offset;
    msg_cloud_pub.row_step = msg_cloud_pub.point_step * msg_cloud_pub.width;
    msg_cloud_pub.data.resize(msg_cloud_pub.row_step * msg_cloud_pub.height);

    auto floatData = reinterpret_cast<float *>(msg_cloud_pub.data.data());
    for (uint32_t i = 0; i < msg_cloud_pub.width - dist_count; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            floatData[i * (msg_cloud_pub.point_step / sizeof(float)) + j] = pointCloud2.at<cv::Vec3f>(i)[j];
        }
    }
}

pcl::PointXYZRGB htl::Converter::pclColor(const size_t &color)
{
    pcl::PointXYZRGB pt;
    switch (color)
    {
    case Converter::Color::BRACK:
        pt = pcl::PointXYZRGB(0, 0, 0);
        break;

    case Converter::Color::WHITE:
        pt = pcl::PointXYZRGB(255, 255, 255);
        break;

    case Converter::Color::SILVER:
        pt = pcl::PointXYZRGB(192, 192, 192);
        break;

    case Converter::Color::BLUE:
        pt = pcl::PointXYZRGB(0, 0, 255);
        break;

    case Converter::Color::RED:
        pt = pcl::PointXYZRGB(255, 0, 0);
        break;

    case Converter::Color::GREEN:
        pt = pcl::PointXYZRGB(0, 255, 0);
        break;

    case Converter::Color::YELLOW:
        pt = pcl::PointXYZRGB(255, 255, 0);
        break;

    case Converter::Color::AQUA:
        pt = pcl::PointXYZRGB(0, 255, 255);
        break;

    case Converter::Color::PINK:
        pt = pcl::PointXYZRGB(255, 0, 255);
        break;

    case Converter::Color::OLIVE:
        pt = pcl::PointXYZRGB(128, 128, 0);
        break;

    case Converter::Color::TEAL:
        pt = pcl::PointXYZRGB(0, 128, 128);
        break;

    case Converter::Color::SKIN:
        pt = pcl::PointXYZRGB(255, 222, 173);
        break;

    default:
        pt = pcl::PointXYZRGB(255, 255, 255);
        break;
    }

    return pt;
}

#endif