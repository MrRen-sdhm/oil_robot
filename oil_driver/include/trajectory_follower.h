//
// Created by sdhm on 7/12/19.
//

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "motor_driver.h"
#include "trajectory.h"

using namespace ros;
using namespace std::chrono;

namespace Oil {

class TrajectoryFollower {

public:
    typedef control_msgs::FollowJointTrajectoryAction Action;
    typedef control_msgs::FollowJointTrajectoryResult Result;
    typedef actionlib::ServerGoalHandle<Action> GoalHandle;

    explicit TrajectoryFollower(MotorDriver &motorDriver, float path_tolerance, float goal_tolerance) :
                        motorDriver_(motorDriver), path_tolerance_(path_tolerance), goal_tolerance_(goal_tolerance) {
        path_tolerance_ = path_tolerance_ * float(M_PI) / 180;
        goal_tolerance_ = goal_tolerance_ * float(M_PI) / 180;
    }

    MotorDriver& motorDriver_;

    static const int joint_count_ = MotorDriver::motor_cnt_;
    static const int max_traj_points_ = 200;

    float path_tolerance_; // 各关节轨迹跟踪位置容差 5度  FIXME:由于通信延迟，此处容差暂时设大
    float goal_tolerance_; // 各关节目标位置容差 0.5度

    // 轨迹执行时间容差
    const uint32_t time_tolerance_ = 1000000000; // 1秒

    // 当前执行的轨迹
    Trajectory <joint_count_, max_traj_points_> current_trajectory_;

    enum class State {
        STARTING,   // 准备启动
        WORKING,    // 正在跟踪
    };

    /* 以下变量同时在中断和主函数中使用 */
    volatile State state_ = State::STARTING; // 当前工作状态
    volatile bool runing_ = false;           // 是否跟随轨迹
    volatile bool follow_done_ = false;      // 轨迹跟踪完成标志
    volatile bool write_once_ = false;       // 目标位置写入标志
    volatile bool last_point_flag_ = false;  // 最后一个轨迹点写入完成标志
    volatile bool path_tolerance_violated = false; // 轨迹执行超过容差标志
    volatile bool goal_tolerance_violated = false; // 终点位置超过容差标志

    TrajectoryPoint <joint_count_> desired_point_{}; // 当前设定的轨迹点
    TrajectoryPoint <joint_count_> actual_point_{};  // 当前实际的轨迹点
    TrajectoryPoint <joint_count_> error_point_{};   // 当前实际的轨迹点

    /* 轨迹跟踪状态 */
    TimePoint time_start_; // 轨迹跟踪的开始时间, 单片机时间
    size_t next_point_index_{}; // 当前插值的下一个轨迹点
    double last_print_time_ = 0; // 轨迹执行状态打印时间

    Result result_; // 轨迹执行结果

    // 线性插值算法
    static int linear_interpolate(const TrajectoryPoint <joint_count_> &begin, const TrajectoryPoint <joint_count_> &end,
                           TrajectoryPoint <joint_count_> &current) {
        // 检查时间戳
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            return -1;
        }
        float percent = float(current.time_nsec - begin.time_nsec) / (end.time_nsec - begin.time_nsec);
        for (int i = 0; i < joint_count_; i++) {
            float delta_pos = end.positions[i] - begin.positions[i];
            float pos = begin.positions[i] + percent * delta_pos;
            current.positions[i] = pos;
        }
        return 0;
    }

    // 设置关节电机目标位姿
    void set_motor_positions(std::array<double, joint_count_> &positions) {
        for (int id = 1; id <= joint_count_; id++) {
            motorDriver_.set_position(id, positions[id-1]);
        }
    }

    // 对比actual与desired, 计算误差error_point_
    void compute_error(const TrajectoryPoint <joint_count_> &desired, const TrajectoryPoint <joint_count_> &actual,
                        TrajectoryPoint <joint_count_> &, bool show_info) {
        for (int i = 0; i < joint_count_; i++) {
            float error = desired.positions[i] - actual.positions[i];
            error_point_.positions[i] = error;
        }
        error_point_.time_nsec = std::max(desired.time_nsec, actual.time_nsec);

        if (show_info) {
            // 获取零位关节角度
            cout << endl << "position_error    [ ";
            for (int i = 1; i <= joint_count_; i++) {
                printf("%0.5f ", error_point_.positions[i - 1]);
            }
            cout << "] rad [ ";
            for (int i = 1; i <= joint_count_; i++) {
                printf("%0.5f ", R2D(error_point_.positions[i - 1]));
            }
            cout << "] deg" << endl;
        }
    }

    // 检查误差是否在容许范围内
    bool is_error_in_tolerance(const TrajectoryPoint <joint_count_> &error, float tolerance, bool display = false) {
        for (int i = 0; i < joint_count_; i++) {
            if (std::abs(error.positions[i]) > tolerance && i < joint_count_ - 1) { // 忽略末端关节
                if (display) {
                    printf("[TRAJ] joint %d: abs(error %f (%f deg)) > tol %f (%f deg)\n", i + 1, std::abs(error.positions[i]),
                            R2D(std::abs(error.positions[i])), tolerance, R2D(tolerance));
                    printf("\033[1;32m[WARN] Please make sure that the motor[%d] could move correctly!\033[0m\n", i+1);
                }
                return false;
            }
        }
        return true;
    }

    // 检查位置与时间容差, 若超出容差则中止执行, 若达到目标则完成执行
    bool check_tolerance(bool show_info) {
        TimePoint now = Clock::now();
        // 计算误差
        compute_error(desired_point_, actual_point_, error_point_, show_info);

        if (next_point_index_ < current_trajectory_.points_length) { // 正在跟踪轨迹点
            if(show_info) printf("\033[1;34m[TRAJ] Follow the trajectory points now.\033[0m\n");
            if (next_point_index_ > 1) { // 不判断起始点
                if (!is_error_in_tolerance(error_point_, path_tolerance_, true)) { // 关节位置误差超过轨迹跟踪容许范围!, 结束执行
                    desired_point_ = actual_point_;
                    desired_point_.time_nsec = 0;
                    // 取消轨迹跟踪
                    cancel();
                    printf("\033[1;32m[WARN] Maybe you should reduce the speed or turn up the tolerance!\033[0m\n");
                    printf("\033[1;34m[TRAJ] PATH_TOLERANCE_VIOLATED.\033[0m\n");
                    path_tolerance_violated = true;

                    return false;
                }
            }
        } else { // 跟踪最后一个轨迹点, 即将结束
            printf("\033[1;34m[TRAJ] Follow the last trajectory point now, trajectory follow will done.\033[0m\n");

            usleep(1000*100); // 休眠0.1s以确保关节已停止运动

            if (!is_error_in_tolerance(error_point_, goal_tolerance_, true)) { // 关节位置误差超过目标位置容许范围, 等待执行, 直到超过时间容差
                uint16_t duration = duration_cast<nanoseconds>(now - time_start_).count();
                if (duration <= current_trajectory_.points[current_trajectory_.points_length - 1].time_nsec + time_tolerance_) {
                    // 用时未超过容差范围, 继续等待
                } else { // 用时超出, 强制停止
                    cancel(); // 取消轨迹跟踪
                    printf("\033[1;34m[TRAJ] GOAL_TOLERANCE_VIOLATED.\033[0m\n");
                    goal_tolerance_violated = true;
                }

                return false;
            }
        }

        return true;
    }

    void spinOnce(bool show_info) {
        // 读当前关节位置
        on_read_pos(show_info);
        // 写目标关节位置
        on_sync_write(show_info);
    }

    // 主循环中写入新位置到虚拟驱动器
    void on_sync_write(bool show_info) {
        if (runing_) {
            uint64_t duration;
            TimePoint now = Clock::now();

            switch (state_) {
                case State::STARTING:
                    time_start_ = now;
                    next_point_index_ = 0;
                    state_ = State::WORKING;
                    printf("\033[1;36m[TRAJ] Start follow trajectory.\033[0m\n");
                    // don't break here

                case State::WORKING:
                    // 计算距离任务开始的时间
                    duration = duration_cast<nanoseconds>(now - time_start_).count();
                    // 找到下一个要跟踪的轨迹点
                    while (duration > current_trajectory_.points[next_point_index_].time_nsec &&
                           next_point_index_ < current_trajectory_.points_length) {
                        next_point_index_++;
                    }

                    // 计算目标位置
                    if (next_point_index_ == 0) {
                        // 起始
                        desired_point_ = current_trajectory_.points[0];
                        // 设定电机状态
                        set_motor_positions(desired_point_.positions);
                    } else if (next_point_index_ < current_trajectory_.points_length) {
                        // 中间状态, 计算插值点
                        desired_point_.time_nsec = duration;
                        // 线性插值
                        linear_interpolate(current_trajectory_.points[next_point_index_ - 1],
                                               current_trajectory_.points[next_point_index_], desired_point_);
                        // 设定电机状态
                        set_motor_positions(desired_point_.positions);
                    } else {
                        // 正在结束
                        desired_point_ = current_trajectory_.points[current_trajectory_.points_length - 1];
                        set_motor_positions(desired_point_.positions);

                        write_once_ = false;
                        last_point_flag_ = true;
                    }

                    // 打印插补后的轨迹
//                    printf("[Traj] servoj[%lu]([%f,%f,%f,%f,%f,%f,%f])\n", next_point_index_, desired_point_.positions[0],
//                             desired_point_.positions[1], desired_point_.positions[2], desired_point_.positions[3],
//                             desired_point_.positions[4], desired_point_.positions[5], desired_point_.positions[6]);

                    // 打印状态
                    if (Time::now().toSec() - last_print_time_ > 0  && show_info) { // 间隔打印, 设为0即一直打印
                        last_print_time_ = Time::now().toSec();
                        auto &p = desired_point_;
                        printf("\033[1;36m[TRAJ]\033[0m %zu/%zu %.3fsec [", next_point_index_,
                               current_trajectory_.points_length, duration / 1e9f);
                        for (int i = 0; i < joint_count_; i++) {
                            if (i > 0)
                                printf(",");
                            printf("%.3f(%.3f)", p.positions[i], R2D(p.positions[i]));
                        }
                        printf("]\n");
                    }
            }

            if (!write_once_) write_once_ = true; // 成功写入一次目标位置, 标志位置位
        }
    }

    // 当读取到新位置时的回调
    void on_read_pos(bool show_info) {
        if (runing_) {
            if(show_info) motorDriver_.print_position(0); // 持续打印关节位置信息
            // 将电机位置反馈记录到actual_point_
            actual_point_.time_nsec = Time::now().nsec;
            actual_point_.positions = motorDriver_.curr_pos;

            // 位置及时间容差检查
            if (write_once_) { // 读取当前位置后, 进行轨迹容差检查, 需在已写入一次目标位置后执行
                if (check_tolerance(show_info)) { // 在误差容许范围内
                    if (last_point_flag_) { // 是最后一个轨迹点
                        last_point_flag_ = false;

                        /// 轨迹执行完成
                        printf("\033[1;36m[TRAJ] Follow trajectory done.\033[0m\n");
                        runing_ = false;
                        state_ = State::STARTING;
                        follow_done_ = true;
                    }
                }
            }
        }
    }

    // 设置要跟踪的目标轨迹
    bool set_goal(GoalHandle& gh) {

        // 检查轨迹消息
        auto parser = TrajectoryParser<joint_count_, max_traj_points_> (motorDriver_.joint_names_, gh, current_trajectory_);

        // 拷贝轨迹数据
        parser.copy_trajectory();

        printf("\033[1;36m[TRAJ] Goal accepted, start trajectory follow.\033[0m\n");

        current_trajectory_.print();

        runing_ = true; /// 置为启动状态

        return true;
    }

    void cancel() {
        runing_ = false; // 取消轨迹跟踪
        state_ = State::STARTING; // 轨迹跟踪设为起始状态
    }
};

}