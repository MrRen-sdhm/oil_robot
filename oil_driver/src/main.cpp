//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"
#include "action_server.h"
#include "jointstate_publisher.h"

using namespace Oil;

int main(int argc, char*argv[]) {

    ros::init(argc, argv, "oil_driver"); // ROS初始化
    ros::NodeHandle nh("~"); // 节点句柄

    // 获取参数
    bool with_hand = false;
    nh.param("with_hand", with_hand, false);
    if (with_hand) printf("\033[1;32m[INFO] Bring up oil with gripper!\033[0m\n");

    bool show_pose_info = false;
    nh.param("show_pose_info", show_pose_info, false);
    if (show_pose_info) printf("\033[1;32m[INFO] Show pose info.\033[0m\n");

    bool show_traj_info = false;
    nh.param("show_traj_info", show_traj_info, false);
    if (show_traj_info) printf("\033[1;32m[INFO] Show pose info.\033[0m\n");

    /// 机械臂驱动、Action服务及轨迹跟踪器实例化
    auto *armDriver = new MotorDriver("192.168.0.6", 502, 1, "joint", show_pose_info); // 机械臂驱动
    Arm::ActionServerArm *action_server_arm(nullptr); // 机械臂Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器
    traj_follower = new TrajectoryFollower(*armDriver, 2.0, 0.2); // 关节轨迹跟随器
    action_server_arm = new Arm::ActionServerArm(*traj_follower, *armDriver, "oil/arm"); // 机械臂Action服务器
    // 开启机械臂Action服务器
    action_server_arm->start();

    /// 手爪驱动及Action服务实例化
    ocservo::OCServoRS485 *handDriver;
    RobotStatePublisher *rs_pub;
    JointStatePublisher *js_pub;
    if (with_hand) {
        handDriver = new ocservo::OCServoRS485(1, "ee_joint1"); // 手抓驱动
        Hand::ActionServerHand *action_server_hand(nullptr); // 手抓Action服务器
        action_server_hand = new Hand::ActionServerHand(*handDriver, "gripper"); // 手抓Action服务器
        // 开启手抓Action服务器
        action_server_hand->start();

        rs_pub = new RobotStatePublisher(*armDriver, *handDriver, "joint_states", 500); // 机器人状态发布器
        // 关节状态发布定时器
        rs_pub->start();
    } else {
        js_pub = new JointStatePublisher(*armDriver, "joint_states", 500); // 关节状态发布器
        // 关节状态发布定时器
        js_pub->start();
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    uint32_t cnt_ = 0;
    while(ros::ok())
    {
//        printf("%d\n", cnt_++); // 运行速度测试, 能达到250HZ

        // 读取关节驱动器中关节位置
        armDriver->do_read_operation();
        // 读当前关节位置, 并写目标位置(轨迹)
        traj_follower->spinOnce(show_traj_info);
        // 目标位置发送给关节驱动器
        armDriver->do_write_operation();
        // 更新轨迹执行状态
        action_server_arm->spinOnce();

        ros::spinOnce();
    }

    spinner.stop();
    ROS_INFO("Exiting oil_driver...");

    return 0;
}

