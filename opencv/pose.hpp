#ifndef _HTL_POSE_HPP__
#define _HTL_POSE_HPP__

#include <opencv2/opencv.hpp>

namespace htl
{
    template <class T>
    struct Quaternion : cv::Vec<T, 4>
    {
    public:
        T x, y, z, w;
    };

    template <class T>
    struct Position : cv::Point3_<T>
    {
    };

    template <class T>
    struct Pose
    {
    public:
        htl::Position<T> position;
        htl::Quaternion<T> quaternion;
    };
} // namespace htl
#endif