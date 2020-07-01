#include <iostream>
#include <algorithm>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

// ransac平面拟合 https://blog.csdn.net/hanshuobest/article/details/72854210

using namespace std;

void plotFrame(pcl::visualization::PCLVisualizer::Ptr &viewer, const Eigen::Vector3d &translation,
               const Eigen::Matrix3d &rotation, const std::string &id, double axis_length) {
    const Eigen::Matrix3d pts = axis_length * rotation;
    const std::string names[3] = {"normal_" + id, "binormal_" + id,
                                  "curvature_" + id};
    const double colors[3][3] = {
            {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    pcl::PointXYZ p;
    p.getVector3fMap() = translation.cast<float>();
    for (int i = 0; i < 3; i++) {
        pcl::PointXYZ q;
        q.getVector3fMap() = (translation + pts.col(i)).cast<float>();
        viewer->addLine<pcl::PointXYZ>(p, q, colors[i][0], colors[i][1], colors[i][2], names[i]);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, names[i]);
    }
}

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

std::vector<float> calCenter(cv::Point& point, const cv::Mat& lookupX, const cv::Mat& lookupY, Eigen::VectorXf coef) {
    printf("point: %d, %d\n", point.x, point.y);
    const float coef_x = lookupX.at<float>(0, point.x); // 像素点与世界点x方向映射关系
    const float coef_y = lookupY.at<float>(0, point.y); // 像素点与世界点y方向映射关系

    std::cout << "coeff_x:" << coef_x << "\ncoeff_y:" << coef_y << std::endl;

    // 平面方程
    // ax+by+cz+d = 0
    // a = cosA, b = cosB, c = cosC, a^2 + b^2 + c^2 = 1, (a,b,c) 即单位方向向量
    // cosA, cosB, cosC 为平面上点(x,y,z)处法向量的方向余弦 |d|为原点到平面的距离

    // 根据相机内参及平面方程计算像素点对应的世界坐标
    // x = coef_x*z
    // y = coef_y*z
    // ->
    // a*coeff_x*z+b*coeff_y*z+c*z+d = 0
    // z = -d/(a*coeff_x+b*coeff_y+c)

    const float a = coef[0], b = coef[1], c = coef[2], d = coef[3];

    float z = -d/(a*coef_x+b*coef_y+c);
    float x = coef_x*z;
    float y = coef_y*z;

    std::cout << "x:" << x << " y:" << y << " z:" << z << std::endl;

    std::vector<float> loc = {x, y, z};
    return loc;
}

int main ()
{
    cv::Rect of_rect; // 加油口外接矩形
    cv::Point of_center; // 加油口中心点

    // 载入图像
    cv::Mat color;
    color = cv::imread("./26_color_1016.jpg");

    // 创建像素点与世界坐标映射关系
    cv::Mat lookupX, lookupY;
    createLookup(lookupX, lookupY, color.cols, color.rows);
//    std::cout << "lookupX:" << lookupX << "\nlookupY:" << lookupY << std::endl;

    /// 检测加油口
    // 均值滤波
    cv::Mat img_blur;
    blur(color, img_blur, cv::Size(10, 10));

    // 灰度转换
    cv::Mat img_gray;
    cvtColor(img_blur, img_gray, cv::COLOR_BGR2GRAY);

    // 霍夫变换圆检测
    vector<cv::Vec3f> circles;
    HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT,1, 1, 80, 60, 40, 120); // 这里的canny算法下限自动设置为上限一半

    // 依次在图中绘制出圆
//    for( size_t i = 0; i < circles.size(); i++ ) {
//        //参数定义
//        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        double radius = cvRound(circles[i][2]);
//        //绘制圆心
//        circle(color, center, 2, cv::Scalar(0,255,0), -1, 8, 0 );
//        //绘制圆轮廓
//        circle(color, center, (int)radius, cv::Scalar(0,0,255), 2, 8, 0 );
//
//        int radius_zoom = (int)(radius*1.1);
//        cv::Rect rect(center.x-radius_zoom, center.y-radius_zoom, 2*radius_zoom, 2*radius_zoom);
//        cv::rectangle(color, rect, cvScalar(0, 255, 255), 2, 8, 0);
//
//        printf("center: %d, %d\n", center.x, center.y);
//        printf("rect: %d, %d, %d, %d\n", rect.x, rect.y, rect.width, rect.height);
//
//        of_center = center; // 获取加油口中心像素坐标
//        of_rect = rect; // 获取加油口外接矩形
//    }

    /// 查找最大圆
    std::vector<double> radius_vec;
    for (int i = 0; i < circles.size(); i++) {
        double radius = cvRound(circles[i][2]);
        radius_vec.push_back(radius);
    }

    auto max_radius = std::max_element(radius_vec.begin(), radius_vec.end());
    auto indice = std::distance(radius_vec.begin(), max_radius);
    std::cout << "最大圆半径是 " << *max_radius << "索引是 " << indice << std::endl << std::endl;

    cv::Point center(cvRound(circles[indice][0]), cvRound(circles[indice][1]));
    double radius = cvRound(circles[indice][2]);
    //绘制圆心
    circle(color, center, 2, cv::Scalar(0,255,0), -1, 8, 0 );
    //绘制圆轮廓
    circle(color, center, (int)radius, cv::Scalar(0,0,255), 2, 8, 0 );

    int radius_zoom = (int)(radius*1.1);
    cv::Rect rect(center.x-radius_zoom, center.y-radius_zoom, 2*radius_zoom, 2*radius_zoom);
    cv::rectangle(color, rect, cvScalar(0, 255, 255), 2, 8, 0);

    printf("center: %d, %d\n", center.x, center.y);
    printf("rect: %d, %d, %d, %d\n", rect.x, rect.y, rect.width, rect.height);

    of_center = center; // 获取加油口中心像素坐标
    of_rect = rect; // 获取加油口外接矩形


    imshow("result", color);

    cv::waitKey(0);

    ////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // 载入点云
    if (pcl::io::loadPCDFile("./26_cloud_1016.pcd", *cloud_origin) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }

    /// 获取加油口无组织无色彩点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = of_rect.y; row < of_rect.y+of_rect.height; row++) {
        for (int col = of_rect.x; col < of_rect.x+of_rect.width; col++) {
            pcl::PointXYZRGBA p_in= cloud_origin->points[row * cloud_origin->width + col];
            pcl::PointXYZ p_out;
            p_out.x = p_in.x;
            p_out.y = p_in.y;
            p_out.z = p_in.z;
            cloud_tmp->points.push_back(p_out);
        }
    }

    /// 统计学滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_tmp);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_tmp);
    std::cout << "Cloud after removing statistical outliers: " << cloud_tmp->size() << std::endl;

    /// 查找加油口最高点
    std::vector<float> p_depth;
    for (int i = 0; i < cloud_tmp->points.size(); i++) {
        const float depth = cloud_tmp->points[i].z;
        if (!isnan(depth) && depth > 0) {
            p_depth.push_back(depth);
        }
    }

    auto min_X = std::min_element(p_depth.begin(), p_depth.end());
    double MinX = *min_X;
    std::cout << "最高点是 " << *min_X << std::endl << std::endl;

    /// 分割最高点附近点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud_tmp->points.size(); i++) {
        const pcl::PointXYZ p = cloud_tmp->points[i];
        if (p.z > 0 && p.z < MinX+0.015) { /// 关键参数
            cloud_of->push_back(p);
        }
    }

    /// 平面拟合
    // 保存局内点索引
    std::vector<int> inliers;
    // 采样一致性模型对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_of));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(0.002); /// 关键参数
    ransac.computeModel();
    ransac.getInliers(inliers);

    Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
    ransac.getModelCoefficients(coef); // 平面方程: ax+by+cz+d = 0

    std::cout << "平面参数:\n" << coef << std::endl; // 平面方程参数

    // 保存局内点
    std::cout << "平面局内点数：" << inliers.size() << std::endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
//    final->resize(inliers.size());
//
//    pcl::copyPointCloud(*cloud_of, inliers, *final);
//    pcl::io::savePCDFile("result.pcd", *final);


    // 显示
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addPointCloud(cloud_of, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->addCoordinateSystem (0.5, "axis", 0);

    // 显示平面
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(coef[0]); // a
    coeffs.values.push_back(coef[1]); // b
    coeffs.values.push_back(coef[2]); // c
    coeffs.values.push_back(coef[3]); // d
    visualizer->addPlane (coeffs, "plane");
    visualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane", 0);

    // 显示中心点位置
    std::vector<float> center_loc = calCenter(of_center, lookupX, lookupY, coef);
    pcl::PointXYZ center_point;
    center_point.x = center_loc[0];
    center_point.y = center_loc[1];
    center_point.z = center_loc[2];
    visualizer->addSphere (center_point, 0.01, 0.0, 1.0, 0.0, "sphere");

    // 显示中心点处坐标系
    double angle_y = atan(coef[0]/coef[2]); // 弧度(-pi/2,pi/2) atan(x/z) 法向量在xz平面投影与z轴夹角
    double angle_x = atan(coef[1]/coef[2]); // 弧度(-pi/2,pi/2) atan(y/z) 法向量在xy平面投影与z轴夹角
    printf("angle_x:%f rad  angle_y:%f rad", angle_x, angle_y);

    Eigen::Vector3d trans(center_point.x, center_point.y, center_point.z);
//    Eigen::Vector3d trans(0,0,0);

    // 绕y轴旋转angle_y, 则法向量在xz平面投影与旋转后的z轴重合
    Eigen::AngleAxisd rot_vector_y (angle_y, Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d rot_matrix_y = rot_vector_y.matrix();

    // 绕x轴旋转-angle_x, 则法向量在yz平面投影与旋转后的z轴重合
    Eigen::AngleAxisd rot_vector_x (-angle_x, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rot_matrix_x = rot_vector_x.matrix();

    // 原始坐标系分别绕x轴和y轴旋转后, 使z轴与平面法向量平行, 作为中心点处坐标系
    Eigen::Matrix3d rot_matrix = rot_matrix_x*rot_matrix_y;

    cout << "rotation matrix =\n" << rot_matrix << endl;

    plotFrame(visualizer, trans, rot_matrix, "frame", 0.1);

    // 显示平面法向量
    pcl::PointXYZ p0, p1;
    p0.x = 0;
    p0.y = 0;
    p0.z = 0;

    if (coef[2] > 0) { // 统一到与z轴同向
        p1.x = coef[0];
        p1.y = coef[1];
        p1.z = coef[2];
    } else {
        p1.x = -coef[0];
        p1.y = -coef[1];
        p1.z = -coef[2];
    }
    visualizer->addLine<pcl::PointXYZ>(p0, p1, 1, 1, 0, "line");


    while(!visualizer->wasStopped()) {
        visualizer->spinOnce(10);
    }
    visualizer->close();

    pcl::PCDWriter writer;
//    writer.writeBinary("oil_filler.pcd", *cloud);

    return 0;
}
