//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <thread>
#include <cmath>

#include "motor_driver.h"
#include "trajectory_follower.h"

#include "ocservo_driver.h"
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommandAction.h>

using namespace ros;

namespace Oil {
    namespace Arm {
        class ActionServerArm { /// 机械臂Action服务
        private:
            typedef control_msgs::FollowJointTrajectoryAction Action;
            typedef control_msgs::FollowJointTrajectoryResult Result;
            typedef actionlib::ServerGoalHandle<Action> GoalHandle;
            typedef actionlib::ActionServer<Action> Server;

            NodeHandle nh_;
            Server as_;

            std::vector<std::string> joint_names_;
            std::set<std::string> joint_set_;
            string action_ns_;

            GoalHandle curr_gh_;
            bool running_;

            TrajectoryFollower &follower_;

            void onGoal(GoalHandle gh);

            void onCancel(GoalHandle gh);

            bool validate(GoalHandle &gh, Result &res);

            bool validateState(GoalHandle &gh, Result &res);

            bool validateJoints(GoalHandle &gh, Result &res);

            bool validateTrajectory(GoalHandle &gh, Result &res);

        public:
            ActionServerArm(TrajectoryFollower &follower, MotorDriver &driver, string action_ns);

            MotorDriver &driver_;

            void start();

            void spinOnce();
        };
    }

    namespace Hand {

        class ActionServerHand { /// 手抓Action服务
        private:
            typedef control_msgs::GripperCommandAction Action;
            typedef control_msgs::GripperCommandActionResult Result;
            typedef actionlib::ServerGoalHandle<Action> GoalHandle;
            typedef actionlib::ActionServer<Action> Server;

            NodeHandle nh_;
            Server as_;

            string action_ns_;

            bool running_ = false;

            std::pair<float, float> position_thresh = {0.0, 0.64};

            const float conversion_coeff_ = 0.64 / 0.04; // 手抓关节 米转换为弧度 系数, 限幅: [0,0.04]m, [0,0.64]rad

            void onGoal(GoalHandle gh);

            void onCancel(GoalHandle gh);

            bool validate(GoalHandle &gh);

        public:
            ActionServerHand(ocservo::OCServoRS485 &driver, string action_ns);

            ocservo::OCServoRS485 &driver_;

            void start();

            void spinOnce();
        };

    }
}