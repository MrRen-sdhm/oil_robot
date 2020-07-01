#include <utility>

#include "oil_filler_pose_realsense.h"

using namespace std;

static std::string getCurrentTimeStr() {
    time_t t = time(nullptr);
    char ch[64] = {0};
//        strftime(ch, sizeof(ch)-1, "%Y-%m-%d %H-%M-%S", localtime(&t));
    strftime(ch, sizeof(ch)-1, "%m%d", localtime(&t));     //年-月-日 时-分-秒
    return ch;
}

void plotFrame(pcl::visualization::PCLVisualizer::Ptr &viewer, const Eigen::Vector3d &translation,
               const Eigen::Matrix3d &rotation, const std::string &id, double axis_length) {
    const Eigen::Matrix3d pts = axis_length * rotation;
    const std::string names[3] = {"normal_" + id, "binormal_" + id, "curvature_" + id};
    const double colors[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
    pcl::PointXYZ p;
    p.getVector3fMap() = translation.cast<float>();
    for (int i = 0; i < 3; i++) {
        pcl::PointXYZ q;
        q.getVector3fMap() = (translation + pts.col(i)).cast<float>();
        viewer->addLine<pcl::PointXYZ>(p, q, colors[i][0], colors[i][1], colors[i][2], names[i]);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, names[i]);
    }
}

OilFillerPose::OilFillerPose(ros::NodeHandle& node, std::string camera_frame, int rate) :
                                                                camera_frame_(std::move(camera_frame)), rate_(rate) {
    printf("Init ....\n");

    /// Realsense cloud and image receiver
    std::string ns = R2_DEFAULT_NS;
    std::string topicColor = R2_TOPIC_IMAGE_COLOR R2_TOPIC_IMAGE_RAW;
    std::string topicDepth = R2_TOPIC_ALIGNED_DEPTH R2_TOPIC_IMAGE_RAW;
    bool useExact = true;
    bool useCompressed = false;
    topicColor = "/" + ns + topicColor;
    topicDepth = "/" + ns + topicDepth;

    receiver = std::make_shared<RealsenseReceiver>(node, topicColor, topicDepth, useExact, useCompressed, rate);

    ROS_INFO("Starting realsense receiver...");
    receiver->run();

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_of = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

void OilFillerPose::imageViewer()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();

    ros::Rate rate(rate_);
    for(; running && ros::ok();) {
        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if (elapsed >= 1.0) {
            fps = frameCount / elapsed;
            oss.str("");
            oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
            start = now;
            frameCount = 0;
        }

        if (!color_draw.empty()) {
            cv::Mat color_show(color_draw);
            if (color_draw.cols > 800) {
                resize(color_show, color_show, cv::Size(0, 0), 0.7, 0.7, cv::INTER_LINEAR);
            }
            cv::putText(color_show, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
            cv::imshow("Image Viewer", color_show);
        }

        int key = cv::waitKey(1);
        switch(key & 0xFF)
        {
            case 27:
            case 'q':
                running = false;
                break;
            case ' ':
            case 's':
                saveCloudAndImages();
                break;
        }

        rate.sleep();
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
}

void OilFillerPose::saveCloudAndImages() {
    std::string baseName, cloudName, colorName, colorDrawName, depthName;

    while (true) {
        oss.str("");
        oss << std::setfill('0') << std::setw(2) << frame;
        baseName = oss.str();
        cloudName = "./" + baseName + "_cloud_" + getCurrentTimeStr() + ".pcd";
        colorName = "./" + baseName + "_color_" + getCurrentTimeStr() + ".jpg";
        colorDrawName = "./" + baseName + "_color_draw_" + getCurrentTimeStr() + ".jpg";
        depthName = "./" + baseName + "_depth_" + getCurrentTimeStr() + ".png";

        if ((access(cloudName.c_str(), 0)) == 0) { // 0已存在,-1不存在
            frame++;
        }
        else {
            break;
        }
    }

    printf("%s\n", ("[INFO] Saving cloud: " + cloudName).c_str());
    writer.writeBinary(cloudName, *cloud);
    printf("%s\n", ("[INFO] Saving color: " + colorName).c_str());
    cv::imwrite(colorName, receiver->color, params);
    printf("%s\n", ("[INFO] Saving color_draw: " + colorDrawName).c_str());
    cv::imwrite(colorDrawName, color_draw, params);
    printf("%s\n", ("[INFO] Saving depth: " + depthName).c_str());
    cv::imwrite(depthName, receiver->depth, params);

    printf("[INFO] Saving complete!\n");

    ++frame;
}

void OilFillerPose::keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    auto *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);

    if(event.keyDown()) {
        switch(event.getKeyCode()) {
            case 27:
            case 'q':
                running = false;
                printf("[ INFO] Quit...\n");
                break;
            case ' ':
            case 's':
                save = true;
                break;
            case 'a':
                if (viewer->contains("ref")) {
                    viewer->removeCoordinateSystem("ref");
                } else {
                    viewer->addCoordinateSystem(0.1, "ref");
                }
                break;
        }
    }
}

void OilFillerPose::publishTF()
{
    Eigen::Quaterniond quat(rot_matrix);
    tf::Quaternion tf_quat;
    tf::quaternionEigenToTF(quat, tf_quat);

    tf::Transform of_tf = tf::Transform(tf_quat, tf::Vector3(trans[0], trans[1], trans[2]));

    broadcaster.sendTransform(tf::StampedTransform(of_tf, ros::Time::now(), camera_frame_, "oil_filler"));
}

bool OilFillerPose::ofDetect() {
    color_draw = receiver->color.clone(); // 获取副本

    if (cloud->points.empty()) {
        printf("[Erro] Origin cloud is empty!\n");
        return false; // 空点云, 跳过
    }

    /// 检测加油口
    // 均值滤波
    cv::Mat img_blur;
    blur(receiver->color, img_blur, cv::Size(10, 10));

    // 灰度转换
    cv::Mat img_gray;
    cvtColor(img_blur, img_gray, cv::COLOR_BGR2GRAY);

    // 霍夫变换圆检测
    vector<cv::Vec3f> circles;
    // 参数:      默认, 最小间距, canny上限, 阈值(越大越圆), 最小半径, 最大半径
//    HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT,1, 1, 80, 60, 40, 120); // 这里的canny算法下限自动设置为上限一半
    HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT,1, 1, 80, 30, 20, 80); // 这里的canny算法下限自动设置为上限一半

    if (circles.empty()) {
        printf("Detect 0 circles!\n");
        return false;
    }

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
    circle(color_draw, center, 4, cv::Scalar(0,255,0), -1, 8, 0 );
    //绘制圆轮廓
    circle(color_draw, center, (int)radius, cv::Scalar(0,0,255), 2, 8, 0 );

    int radius_zoom = (int)(radius*1.2); // 放大矩形框
    cv::Rect rect(center.x-radius_zoom, center.y-radius_zoom, 2*radius_zoom, 2*radius_zoom);
    cv::rectangle(color_draw, rect, cvScalar(0, 255, 255), 2, 8, 0);

    if (rect.x < 0 || rect.x > receiver->color.cols || rect.y < 0 || rect.y > receiver->color.rows) {
        printf("[Erro] Bad rect!\n");
        return false;
    }

    printf("center: %d, %d\n", center.x, center.y);
    printf("rect: %d, %d, %d, %d\n", rect.x, rect.y, rect.width, rect.height);

    of_center = center; // 获取加油口中心像素坐标
    of_rect = rect; // 获取加油口外接矩形

//    imshow("result", color_draw);
//    cv::waitKey(0);

    return true;
}

bool OilFillerPose::ofPlaneCal() {
    /// 获取加油口无组织无色彩点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = of_rect.y; row < of_rect.y+of_rect.height; row++) {
        for (int col = of_rect.x; col < of_rect.x+of_rect.width; col++) {
            pcl::PointXYZRGBA p_in= cloud->points[row * cloud->width + col];
            pcl::PointXYZ p_out;
            p_out.x = p_in.x;
            p_out.y = p_in.y;
            p_out.z = p_in.z;
            cloud_tmp->points.push_back(p_out);
        }
    }

    printf("cloud_tmp size:%zu\n", cloud_tmp->points.size());
    if (cloud_tmp->points.empty()) {
        printf("[Erro] Cloud_tmp is empty! no point in selected area.\n");
        return false;
    }

    /// 统计学滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_tmp);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_tmp);

    /// 查找加油口最高点
    std::vector<float> p_depth;
    for (int i = 0; i < cloud_tmp->points.size(); i++) {
        const float depth = cloud_tmp->points[i].z;
        if (!isnan(depth) && depth > 0) {
            p_depth.push_back(depth);
        }
    }

    if (p_depth.empty()) {
        printf("[Erro] Could not get min_depth!\n");
//        writer.writeBinary("/home/sdhm/cloud_tmp.pcd", *cloud_tmp);
        return false;
    }
    auto min_depth = std::min_element(p_depth.begin(), p_depth.end());
    double MinDepth = *min_depth;
    std::cout << "最高点是 " << *min_depth << std::endl << std::endl;

    if (isnan(MinDepth)) {
        printf("[Erro] 最高点无效!\n");
//        writer.writeBinary("/home/sdhm/cloud_tmp.pcd", *cloud_tmp);
        return false; // 无效最高点
    }

    /// 分割最高点附近点云
    cloud_of->clear();
    for (int i = 0; i < cloud_tmp->points.size(); i++) {
        const pcl::PointXYZ p = cloud_tmp->points[i];
        if (p.z > 0 && p.z < MinDepth+0.03) { /// 关键参数
            cloud_of->push_back(p);
        }
    }

    /// 平面拟合
    // 保存局内点索引
    std::vector<int> inliers;
    // 采样一致性模型对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_of));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(0.0015); /// 关键参数
    ransac.computeModel();
    ransac.getInliers(inliers);

    ransac.getModelCoefficients(coef); // 平面方程: ax+by+cz+d = 0

    std::cout << "平面局内点数：" << inliers.size() << std::endl;
    if (inliers.size() < 1500) { // TODO:阈值可根据平面距离调整
        printf("[Erro] Too few points in cloud_of!\n");
        return false;
    }

    std::cout << "平面参数:\n" << coef << std::endl; // 平面方程参数

    return true;
}

void OilFillerPose::ofCenterCal() {
    const float coef_x = receiver->lookupX.at<float>(0, of_center.x); // 像素点与世界点x方向映射关系
    const float coef_y = receiver->lookupY.at<float>(0, of_center.y); // 像素点与世界点y方向映射关系

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

    trans << x, y, z; // 记录加油口坐标系平移矩阵
    cout << "trans =\n" << trans << endl;

}

void OilFillerPose::ofPoseCal() {
    double angle_y = atan(coef[0]/coef[2]); // 弧度(-pi/2,pi/2) atan(x/z) 法向量在xz平面投影与z轴夹角
    double angle_x = atan(coef[1]/coef[2]); // 弧度(-pi/2,pi/2) atan(y/z) 法向量在xy平面投影与z轴夹角
//    printf("angle_x:%f rad  angle_y:%f rad", angle_x, angle_y);

    // 绕y轴旋转angle_y, 则法向量在xz平面投影与旋转后的z轴重合
    Eigen::AngleAxisd rot_vector_y (angle_y, Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d rot_matrix_y = rot_vector_y.matrix();

    // 绕x轴旋转-angle_x, 则法向量在yz平面投影与旋转后的z轴重合
    Eigen::AngleAxisd rot_vector_x (-angle_x, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rot_matrix_x = rot_vector_x.matrix();

    // 原始坐标系分别绕x轴和y轴旋转后, 使z轴与平面法向量平行, 作为中心点处坐标系
    rot_matrix = rot_matrix_x*rot_matrix_y;

    cout << "rot_matrix =\n" << rot_matrix << endl;
}

void OilFillerPose::ofPoseShow(pcl::visualization::PCLVisualizer::Ptr &visualizer) {

    // 显示平面
//    pcl::ModelCoefficients coeffs;
//    coeffs.values.push_back(coef[0]); // a
//    coeffs.values.push_back(coef[1]); // b
//    coeffs.values.push_back(coef[2]); // c
//    coeffs.values.push_back(coef[3]); // d
//    visualizer->addPlane (coeffs, "plane");
//    visualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane", 0);

    // 显示平面法向量
//    pcl::PointXYZ p0, p1;
//    p0.x = 0;
//    p0.y = 0;
//    p0.z = 0;
//
//    if (coef[2] > 0) { // 统一到与z轴同向
//        p1.x = coef[0];
//        p1.y = coef[1];
//        p1.z = coef[2];
//    } else {
//        p1.x = -coef[0];
//        p1.y = -coef[1];
//        p1.z = -coef[2];
//    }
//    visualizer->addLine<pcl::PointXYZ>(p0, p1, 1, 1, 0, "line");
//    visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "line");

    // 显示中心点位置
    pcl::PointXYZ center_point;
    center_point.x = trans[0];
    center_point.y = trans[1];
    center_point.z = trans[2];
    visualizer->addSphere (center_point, 0.005, 0.0, 1.0, 0.0, "sphere");

    // 显示加油口姿态
    plotFrame(visualizer, trans, rot_matrix, "frame", 0.06);
}

void OilFillerPose::run(int loop_rate) {
    ros::Rate rate(loop_rate); // 与采集频率接近即可
    while(ros::ok()) {
        pcl::copyPointCloud(*receiver->cloud, *cloud); // copy原始点云

        if (ofDetect() && ofPlaneCal()) { // 检测到加油口, 平面拟合成功
            ofCenterCal(); // 加油口中心坐标计算
            ofPoseCal(); // 加油口姿态解算
            publishTF(); // 发布加油口姿态
        }

        ros::spinOnce();
        rate.sleep();
    }

    printf("[INFO] Exit oil filter detector...\n");
    receiver->stop();
}

void OilFillerPose::runShow(int loop_rate) {
    running = true;
    // 启动图像显示线程
    imageViewerThread = std::thread(&OilFillerPose::imageViewer, this);
    imageViewerThread.detach(); // 将子线程从主线程里分离

    // PCLVisualizer初始化
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&OilFillerPose::keyboardEvent, *this, (void *)visualizer.get());

    ros::Rate rate(loop_rate); // 与采集频率接近即可
    while (ros::ok() && running && !visualizer->wasStopped()) {
        visualizer->removeAllShapes();

        pcl::copyPointCloud(*receiver->cloud, *cloud); // copy原始点云

        if (ofDetect() && ofPlaneCal()) { // 检测到加油口, 平面拟合成功
            ofCenterCal(); // 加油口中心坐标计算
            ofPoseCal(); // 加油口姿态解算
            ofPoseShow(visualizer); // 显示加油口姿态
            publishTF(); // 发布加油口姿态
        }

        // 更新点云显示
        visualizer->updatePointCloud(cloud, cloudName);

        if (save) { // 保存点云及结果
            saveCloudAndImages();
            save = false;
        }

        visualizer->spinOnce(10);

        ros::spinOnce();
        rate.sleep();
    }

    printf("[INFO] Exit oil filter detector...\n");
    visualizer->close();
    receiver->stop();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oil_filler_pose");
    ros::NodeHandle node("~");

    // 参数解析
    std::string camera_frame;
    node.param("camera_frame", camera_frame, std::string("camera_color_optical_frame"));

    bool show;
    node.param("show", show, true);

    if(!ros::ok()) {
        return 1;
    }

    int loop_rate = 15; // 与采集频率接近即可

    OilFillerPose of_pose(node, camera_frame, loop_rate);
    if (!show) {
        of_pose.run(loop_rate);
    } else {
        of_pose.runShow(loop_rate);
    }

    ros::shutdown();
    return 0;
}