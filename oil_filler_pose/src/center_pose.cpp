//
// Created by sdhm on 10/14/19.
//

#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

void readCameraInfo(cv::Mat &cameraMatrix)
{
    // realsense
    std::vector<double> cameraMatrixVec = {915.8143310546875, 0.0, 643.5238037109375,
                                           0.0, 916.0640869140625, 369.673583984375,
                                           0.0, 0.0, 1.0};

    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
        *itC = cameraMatrixVec[i];
    }
}

void createLookup(cv::Mat &lookupX, cv::Mat &lookupY, size_t width, size_t height)
{
    cv::Mat cameraMatrixColor;
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

    readCameraInfo(cameraMatrixColor);

    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}

int main(int argc, char** argv)
{
    /// 创建点云
    cv::Mat lookupX, lookupY;

    createLookup(lookupX, lookupY, 1280, 720);

    std::cout << "lookupX:" << lookupX << "\nlookupY:" << lookupY << std::endl;

    const float coef_y = lookupY.at<float>(0, 313);
    const float coef_x = lookupX.at<float>(0, 818);

    std::cout << "coeff_x:" << coef_x << "\ncoeff_y:" << coef_y << std::endl; // coeff_x:0.190515 coeff_y:-0.0618664

    // 平面方程
    // ax+by+cz+d = 0
    // a = cosA, b = cosB, c = cosC, a^2 + b^2 + c^2 = 1, (a,b,c) 即单位方向向量
    // cosA, cosB, cosC 为平面上点(x,y,z)处法向量的方向余弦 |d|为原点到平面的距离

    // 根据相机内参及平面方程计算像素点对应的世界坐标
    // x = coef_x*z
    // y = coef_y*z
    // ->
    // a*coeff_x*z+b*coeff_y*z+c*z = d
    // z = d/(a*coeff_x+b*coeff_y+c)

    const float a = 0.171367, b = -0.161405, c = -0.971896, d = -0.256219;

    float z = d/(a*coef_x+b*coef_y+c);
    float x = coef_x*z;
    float y = coef_y*z;

    std::cout << "x:" << x << " y:" << y << " z:" << z << std::endl; // x:0.0525293 y:-0.017058 z:0.275723

    return 0;
}






