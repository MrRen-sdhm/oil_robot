#include <iostream>
#include <algorithm>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

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

int main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // load the file
    if (pcl::io::loadPCDFile("../../data/05_cloud_1014.pcd", *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }

    pcl::copyPointCloud(*cloud, *cloud_origin);

    std::vector<float> depth;

    for (int row = 0; row < 720; row++) {
        for (int col = 640; col < 1280; col++) {
            depth.push_back(cloud->points[row * cloud->width + col].z);
        }
    }

//    for(int i=0; i < depth.size()/5; ++i)
//        std::cout << depth[i] << ' ';


    auto min_X = std::min_element(depth.begin(), depth.end());

    double MinX = *min_X;

    auto distance = std::distance(depth.begin(), min_X);
    std::cout << "最高点是 " << *min_X << std::endl << std::endl;

    // 获取加油口有组织点云
    for (int row = 0; row < 720; row++) {
        for (int col = 0; col < 1280; col++) {
            float depth_ = cloud->points[row * cloud->width + col].z;
            if (depth_ > 0 && depth_ < MinX+0.01) { // 截取最高点附近的点云
//                printf("%.3f ", depth_);
            } else {
                cloud->points[row * cloud->width + col].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    for (int row = 0; row < 720; row++) {
        for (int col = 0; col < 640; col++) {
            cloud->points[row * cloud->width + col].z = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // 获取加油口无组织无色彩点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of (new pcl::PointCloud<pcl::PointXYZ>);

    for (int row = 0; row < 720; row++) {
        for (int col = 0; col < 1280; col++) {
            pcl::PointXYZRGBA p1= cloud->points[row * cloud->width + col];
            float depth_ = p1.z;
            if (depth_ > 0 && depth_ < MinX+0.01) { // 截取最高点附近的点云
                pcl::PointXYZ p2;
                p2.x = p1.x;
                p2.y = p1.y;
                p2.z = p1.z;
                cloud_of->points.push_back(p2);
            }
        }
    }

    // 获取加油口无组织有色彩点云
//    std::vector<int> indices;
//    for (int i = 0; i < cloud->size(); i++) {
//        const pcl::PointXYZRGBA &p = cloud->points[i];
//        if (p.z > 0 && p.z < MinX+0.01) {
//        } else {
//            indices.push_back(i);
//        }
//    }
//
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_of (new pcl::PointCloud<pcl::PointXYZRGBA>);
//    cloud_of->points.resize(indices.size());
//    for (int i = 0; i < indices.size(); i++) {
//        cloud_of->points[i] = cloud->points[indices[i]];
//    }

    // 平面拟合
    // 保存局内点索引
    std::vector<int> inliers;
    // 采样一致性模型对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_of));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(0.001);
    ransac.computeModel();
    ransac.getInliers(inliers);

    std::cout << "局内点：" << inliers.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    final->resize(inliers.size());

    pcl::copyPointCloud(*cloud_of, inliers, *final);
    pcl::io::savePCDFile("result.pcd", *final);

    Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
    ransac.getModelCoefficients(coef);

    std::cout << coef << std::endl; // 平面方程参数

    // 显示
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_of, 255, 0, 0); // 红
    visualizer->addPointCloud(cloud_origin, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->addCoordinateSystem (0.5, "axis", 0);

    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(coef[0]);
    coeffs.values.push_back(coef[1]);
    coeffs.values.push_back(coef[2]);
    coeffs.values.push_back(coef[3]);
    visualizer->addPlane (coeffs, "plane");
    visualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane", 0);

    // 显示中心点位置
    pcl::PointXYZ center_point;
    center_point.x = 0.0525293;
    center_point.y = -0.017058;
    center_point.z = 0.275723;
    visualizer->addSphere (center_point, 0.01, 0.0, 1.0, 0.0, "sphere");

    // 显示中心点处坐标系
    double angle_y = atan(coef[0]/coef[2]); // 弧度(-pi/2,pi/2) atan(x/z) 法向量在xz平面投影与z轴夹角
    double angle_x = atan(coef[1]/coef[2]); // 弧度(-pi/2,pi/2) atan(y/z) 法向量在xy平面投影与z轴夹角
    printf("angle_x:%f rad  angle_y:%f rad", angle_x, angle_y);

    Eigen::Vector3d trans(center_point.x, center_point.y, center_point.z);
//    Eigen::Vector3d trans(0,0,0);
    Eigen::AngleAxisd rot_vector_y (angle_y, Eigen::Vector3d(0, 1, 0)); // 绕y轴旋转angle_y, 则法向量在xz平面投影与旋转后的z轴重合
    Eigen::Matrix3d rot_matrix_y = rot_vector_y.matrix();

    Eigen::AngleAxisd rot_vector_x (-angle_x, Eigen::Vector3d(1, 0, 0)); // 绕x轴旋转-angle_x, 则法向量在yz平面投影与旋转后的z轴重合
    Eigen::Matrix3d rot_matrix_x = rot_vector_x.matrix();

    Eigen::Matrix3d rot_matrix = rot_matrix_x*rot_matrix_y;

    cout << "rotation matrix =\n" << rot_matrix << endl;

    plotFrame(visualizer, trans, rot_matrix, "frame", 0.1);

    pcl::PointXYZ p0, p1;
    p0.x = 0;
    p0.y = 0;
    p0.z = 0;
    p1.x = -coef[0];
    p1.y = -coef[1];
    p1.z = -coef[2];
    visualizer->addLine<pcl::PointXYZ>(p0, p1, 1, 1, 0, "line");

    while(!visualizer->wasStopped()) {
        visualizer->spinOnce(10);
    }
    visualizer->close();

    pcl::PCDWriter writer;
    writer.writeBinary("oil_filler.pcd", *cloud);

    return 0;
}
