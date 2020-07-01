//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cstdlib>
#include <vector>

#include "motor_driver.h"
#include "ocservo_driver.h"

using namespace ros;


namespace Oil {
class RobotStatePublisher {
public:
    explicit RobotStatePublisher(MotorDriver& arm_driver, ocservo::OCServoRS485& hand_driver, const string& topic_name, int timer_span) :
            timer_span_(timer_span), joint_pub_( nh_.advertise<sensor_msgs::JointState>(topic_name, 1)), arm_driver_(arm_driver), hand_driver_(hand_driver) {
    }

    void start();
    void pub_callback(const ros::TimerEvent& e);

private:
    /// 定时器相关参数
    ros::Timer timer_;
    ros::NodeHandle nh_;
    int timer_span_; // 通信频率 HZ

    Publisher joint_pub_;

    MotorDriver& arm_driver_;
    ocservo::OCServoRS485& hand_driver_;

    bool publishJoints(Time &t);
};

}

namespace Oil {
class JointStatePublisher {
public:
    explicit JointStatePublisher(MotorDriver& arm_driver, const string& topic_name, int timer_span) :
            timer_span_(timer_span), joint_pub_( nh_.advertise<sensor_msgs::JointState>(topic_name, 1)), arm_driver_(arm_driver) {
    }

    void start();
    void pub_callback(const ros::TimerEvent& e);

private:
    /// 定时器相关参数
    ros::Timer timer_;
    ros::NodeHandle nh_;
    int timer_span_; // 通信频率 HZ

    Publisher joint_pub_;

    MotorDriver& arm_driver_;

    bool publishJoints(Time &t);
};

}

namespace Oil {
    class GripperStatePublisher {
    public:
        explicit GripperStatePublisher(ocservo::OCServoRS485& hand_driver, const string& topic_name, int timer_span) :
                timer_span_(timer_span), joint_pub_(nh_.advertise<sensor_msgs::JointState>(topic_name, 1)), hand_driver_(hand_driver) {
        }

        void start();
        void pub_callback(const ros::TimerEvent& e);

    private:
        /// 定时器相关参数
        ros::Timer timer_;
        ros::NodeHandle nh_;
        int timer_span_; // 通信频率 HZ

        Publisher joint_pub_;

        ocservo::OCServoRS485& hand_driver_;

        bool publishGripper(Time &t);
    };

}