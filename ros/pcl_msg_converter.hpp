#ifndef HTL_PCL_MSG_CONVERTER__
#define HTL_PCL_MSG_CONVERTER__

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace htl
{
    class PCL_Converter
    {
    public:
        static void cvMat_to_msgPointCloud(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, const size_t &color);
        static void cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count = 0);
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

void htl::PCL_Converter::cvMat_to_msgPointCloud(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, const size_t &color)
{

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    for (int i = 0; i < pointCloud2.rows; i++)
    {
        //　色
        pcl::PointXYZRGB pt;
        pt = PCL_Converter::pclColor(color);
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

void htl::PCL_Converter::cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count)
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

pcl::PointXYZRGB htl::PCL_Converter::pclColor(const size_t &color)
{
    pcl::PointXYZRGB pt;
    switch (color)
    {
    case Color::BRACK:
        pt = pcl::PointXYZRGB(0, 0, 0);
        break;

    case Color::WHITE:
        pt = pcl::PointXYZRGB(255, 255, 255);
        break;

    case Color::SILVER:
        pt = pcl::PointXYZRGB(192, 192, 192);
        break;

    case Color::BLUE:
        pt = pcl::PointXYZRGB(0, 0, 255);
        break;

    case Color::RED:
        pt = pcl::PointXYZRGB(255, 0, 0);
        break;

    case Color::GREEN:
        pt = pcl::PointXYZRGB(0, 255, 0);
        break;

    case Color::YELLOW:
        pt = pcl::PointXYZRGB(255, 255, 0);
        break;

    case Color::AQUA:
        pt = pcl::PointXYZRGB(0, 255, 255);
        break;

    case Color::PINK:
        pt = pcl::PointXYZRGB(255, 0, 255);
        break;

    case Color::OLIVE:
        pt = pcl::PointXYZRGB(128, 128, 0);
        break;

    case Color::TEAL:
        pt = pcl::PointXYZRGB(0, 128, 128);
        break;

    case Color::SKIN:
        pt = pcl::PointXYZRGB(255, 222, 173);
        break;

    default:
        pt = pcl::PointXYZRGB(255, 255, 255);
        break;
    }

    return pt;
}

#endif