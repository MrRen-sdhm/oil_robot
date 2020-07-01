//
// Created by sdhm on 7/12/19.
//

#pragma once

#include <array>
#include <cmath>
#include <utility>
#include <functional>
#include <limits>
#include <utility>

#include <control_msgs/FollowJointTrajectoryGoal.h>

namespace Oil {

template <size_t joint_count>
struct JointTolerance {
    std::array<float, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
};

// 一个轨迹点
template <size_t joint_count>
struct TrajectoryPoint {
    std::array<double, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
    //        std::array<float, joint_count> effort;
    uint64_t time_nsec; // 相对于轨迹跟踪开始的时间偏移
};

// 轨迹
template <size_t joint_count, size_t max_trajectory_points>
struct Trajectory {
    static const int joint_count_ = joint_count;
    // 待执行的轨迹
    std::array<TrajectoryPoint<joint_count>, max_trajectory_points> points;
    // 待执行的轨迹长度
    size_t points_length = 0;

    void print() {
        if (points_length <= 0) {
            printf("[TRAJ Empty]\n");
            return;
        }
        printf("[TRAJ] Trajectory length=%zu (%.3f s)]\n", points_length, (points[points_length - 1].time_nsec - points[0].time_nsec) / 1e9f);

        auto& p = points[0];
        printf("[TRAJ] begin (%d ms)=[", int(p.time_nsec / 1000000));
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", p.positions[i], R2D(p.positions[i]));
        }
        printf("]\n");

        auto& p1 = points[points_length - 1];
        printf("[TRAJ] end (%d ms)=[", int(p1.time_nsec / 1000000));
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", p1.positions[i], R2D(p1.positions[i]));
        }
        printf("]\n");
    }
};

template <size_t joint_count, size_t max_trajectory_points>
class TrajectoryParser {
public:
    typedef control_msgs::FollowJointTrajectoryAction Action;
    typedef control_msgs::FollowJointTrajectoryResult Result;
    typedef actionlib::ServerGoalHandle<Action> GoalHandle;

    std::vector<string> joint_names;

    const GoalHandle& gh;

    Trajectory<joint_count, max_trajectory_points>& traj; // 待执行的轨迹

    TrajectoryParser(std::vector<string>  _joint_names, decltype(gh) _gh, decltype(traj) _traj)
            : joint_names(std::move(_joint_names)), gh(_gh) ,traj(_traj) {
    }

    // 从JointTrajectory消息中拷贝轨迹数据, 并将环形关节的位置设定值限制为 -pi 到 pi
    // id_to_index由_check_trajectory计算得到
    void copy_trajectory() {
        auto goal = gh.getGoal();
        // 拷贝数据
        for (int i = 0; i < goal->trajectory.points.size(); i++) {
            const trajectory_msgs::JointTrajectoryPoint& p = goal->trajectory.points[i];
            // 相对于轨迹跟踪开始的时间偏移 单位ns
            traj.points[i].time_nsec = (uint64_t)p.time_from_start.sec * 1000000000 + p.time_from_start.nsec;

            for (int j = 0; j < joint_count; j++) {
                double pos = p.positions[j];
                traj.points[i].positions[j] = pos;
            }
        }
        traj.points_length = goal->trajectory.points.size();
    }
};

}
