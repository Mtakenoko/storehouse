#include "/home/takeyama/workspace/htl/opencv/triangulate.hpp"

#define FOCAL_X 396.7
#define FOCAL_Y 396.9
#define PP_X 160
#define PP_Y 160

int main(int argc, char **argv)
{
    // オブジェクトの位置
    std::vector<cv::Point3f> objectPoints;
    cv::Point3f object_point(0.0, 3.0, 10.0);
    objectPoints.push_back(object_point);

    // カメラの内部パラメータ
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << FOCAL_X, 0.0, PP_X,
                            0.0, FOCAL_Y, PP_Y,
                            0.0, 0.0, 1.0);
    cv::Mat distcoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // 透視射影
    cv::Mat Rot = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    cv::Mat rvec;
    cv::Rodrigues(Rot.t(), rvec);
    cv::Mat left_tvec = (cv::Mat_<float>(3, 1) << 0., 0, 0.0);
    cv::Mat right_tvec = (cv::Mat_<float>(3, 1) << 0, 0, -1.0);
    std::vector<cv::Point2f> left_image, right_image;
    cv::projectPoints(objectPoints, rvec, -Rot.t() * left_tvec, cameraMatrix, distcoeffs, left_image);
    cv::projectPoints(objectPoints, rvec, -Rot.t() * right_tvec, cameraMatrix, distcoeffs, right_image);
    std::cout << "left_im  : " << left_image[0] << std::endl;
    std::cout << "Right_im : " << right_image[0] << std::endl;
    std::vector<cv::Point2f> point2D;
    // left_image[0].x += 1.0;
    // left_image[0].y += 1.0;
    point2D.push_back(left_image[0]);
    point2D.push_back(right_image[0]);

    // 透視射影行列の生成
    std::vector<cv::Mat> ProjectionMatrix;
    cv::Mat left_cameraPose, right_cameraPose;
    cv::hconcat(Rot.t(), -Rot.t() * left_tvec, left_cameraPose);
    cv::hconcat(Rot.t(), -Rot.t() * right_tvec, right_cameraPose);
    cv::Mat left_ProjMat = cameraMatrix * left_cameraPose;
    ProjectionMatrix.push_back(left_ProjMat);
    cv::Mat right_ProjMat = cameraMatrix * right_cameraPose;
    ProjectionMatrix.push_back(right_ProjMat);

    // for (int i = 0; i < 10; i++)
    // {
    //     // 透視射影
    //     float theta = M_PI / 180.0 * ((float)i * 2.0);
    //     cv::Mat Rot_2 = (cv::Mat_<float>(3, 3) << (float)std::cos(theta), 0.0, (float)std::sin(theta),
    //                      0.0, 1.0, 0.0,
    //                      -(float)std::sin(theta), 0.0, (float)std::cos(theta));
    //     cv::Mat rvec_2;
    //     cv::Rodrigues(Rot_2.t(), rvec_2);
    //     cv::Mat tvec_2 = (cv::Mat_<float>(3, 1) << 0.1, 0.0, 0.0);
    //     std::vector<cv::Point2f> image_2;
    //     cv::projectPoints(objectPoints, rvec_2, -Rot_2.t() * tvec_2, cameraMatrix, distcoeffs, image_2);
    //     std::cout << "im_2  : " << image_2[0] << std::endl;
    //     point2D.push_back(image_2[0]);

    //     // 透視射影行列の生成
    //     cv::Mat cameraPose_2;
    //     cv::hconcat(Rot_2.t(), -Rot_2.t() * rvec_2, cameraPose_2);
    //     cv::Mat ProjMat_2 = cameraMatrix * cameraPose_2;
    //     ProjectionMatrix.push_back(ProjMat_2);
    // }

    cv::Mat calcPoint = htl::Triangulate::triangulation<float>(point2D, ProjectionMatrix);
    std::cout << "Point3D  : " << Rot * calcPoint << std::endl;
    cv::Mat calcPoint_RANSAC = htl::Triangulate::triangulation_RANSAC<float>(point2D, ProjectionMatrix);
    std::cout << "Point3D_RANSAC  : " << calcPoint_RANSAC << std::endl;
}