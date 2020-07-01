//
// Created by sdhm on 7/7/19.
//

#include "jointstate_publisher.h"

namespace Oil {
bool RobotStatePublisher::publishJoints(Time &t) {
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = t;

    // arm
    joint_msg.name.assign(arm_driver_.joint_names_.begin(), arm_driver_.joint_names_.end());
    joint_msg.position.assign(arm_driver_.curr_pos.begin(), arm_driver_.curr_pos.end());
    joint_msg.velocity.assign(arm_driver_.curr_vel.begin(), arm_driver_.curr_vel.end());
    joint_msg.effort.assign(arm_driver_.curr_eff.begin(), arm_driver_.curr_eff.end());

    // hand
    joint_msg.name.push_back(hand_driver_.joint_name_);
    joint_msg.position.push_back(hand_driver_.curr_position_in_m_);
    joint_msg.velocity.push_back(0.1);
    joint_msg.effort.push_back(0.1);

    joint_pub_.publish(joint_msg);

    return true;
}

void RobotStatePublisher::start() {
    // 通信定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_span_), &RobotStatePublisher::pub_callback, this); // 50HZ 受限于主循环频率
    timer_.start();
}

// 定时发布回调函数
void RobotStatePublisher::pub_callback(const ros::TimerEvent& e) {
    Time time = Time::now();
    // 写各关节目标位置
    publishJoints(time);
}

}

namespace Oil {
bool JointStatePublisher::publishJoints(Time &t) {
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = t;

    // arm
    joint_msg.name.assign(arm_driver_.joint_names_.begin(), arm_driver_.joint_names_.end());
    joint_msg.position.assign(arm_driver_.curr_pos.begin(), arm_driver_.curr_pos.end());
    joint_msg.velocity.assign(arm_driver_.curr_vel.begin(), arm_driver_.curr_vel.end());
    joint_msg.effort.assign(arm_driver_.curr_eff.begin(), arm_driver_.curr_eff.end());

    joint_pub_.publish(joint_msg);

    return true;
}

void JointStatePublisher::start() {
    // 通信定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_span_), &JointStatePublisher::pub_callback, this); // 50HZ 受限于主循环频率
    timer_.start();
}

// 定时发布回调函数
void JointStatePublisher::pub_callback(const ros::TimerEvent& e) {
    Time time = Time::now();
    // 写各关节目标位置
    publishJoints(time);
}

}

namespace Oil {
// 发布执行器状态
bool GripperStatePublisher::publishGripper(Time &t) {
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = t;
    joint_msg.name.push_back(hand_driver_.joint_name_);
    joint_msg.position.push_back(hand_driver_.curr_position_in_m_);
    joint_msg.velocity.push_back(0.1);
    joint_msg.effort.push_back(0.1);
    joint_pub_.publish(joint_msg);
    return true;
}

void GripperStatePublisher::start() {
    // 通信定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_span_), &GripperStatePublisher::pub_callback, this); // 50HZ 受限于主循环频率
    timer_.start();
}

// 定时发布回调函数
void GripperStatePublisher::pub_callback(const ros::TimerEvent& e) {
    Time time = Time::now();
    publishGripper(time);
}

}
