//
// Created by sdhm on 7/5/19.
//

#include "motor_driver.h"

namespace Oil {
MotorDriver::MotorDriver(string ip, const int port, int slaver, const string& joint_prefix, bool show_info) :
                                                                            slaver_(slaver), show_info_(show_info) {
    // modbus tcp 初始化
    m_master_ = new ModbusAdapter();
    m_master_->modbusConnectTCP(std::move(ip), port);

    while (true) { // 循环等待控制器初始化
        if (!init_ready_flag_) { // 判断是否初始化完毕

            // 读取所有控制器寄存器, 并存入OilDevice Register
            if (!m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, (uint16_t *) device_.memory())) {
                throw runtime_error("\033[1;31mFail to read all register from controller!\033[0m\n");
            }

            if (init_ready_flag_) { // 控制器已初始化完毕
                sleep(1); // 休眠1s以确保控制器初始化完毕
                // 判断当前位置寄存器与目标位置寄存器是否相同
                for (int id = 1; id <= motor_cnt_; id++) {
                    // 启动时当前位置与目标位置差值过大, 说明: 1、控制器中目标位置存在错误, 须重启控制器 2、电机当前未使能, 导致各关节角度偏差, 将误差阈值调大即可
                    if (std::abs(device_.get_goal_position(id) - device_.get_curr_position(id)) > 0.005) {
                        printf("[WARN] Id[%d] goal_position=%f curr_position=%f\n", id, device_.get_goal_position(id), device_.get_curr_position(id));
                        throw runtime_error("\033[1;31mGoal positions is not equal to current positions!\nPlease restart the controller!\033[0m\n");
                    }
                }

                // 判断零位寄存器是否有值, 无值便发出警告, 须设置零位
                if (std::abs(std::accumulate(zero_pos.begin(), zero_pos.end(), 0.0l)) < 0.001) {
                    ROS_ERROR("Zero position registers are empty now, please set the zero positions through hmi!");
                }
            }

            if (device_.registers.ctrller_status == 1) {
                init_ready_flag_ = true;
                break;
            } else {
                printf("[INFO] Controller is initializing...\n"); // 控制器初始化完毕
            }
        }
    }

    // 读取位运算寄存器, 调试用
//    if (!m_master_->modbusReadHoldReg(slaver_, hmi_bit_addr_head_, bit_reg_len_, (uint16_t *) device_.bit_memory())) {
//        throw runtime_error("\033[1;31mFail to read bit register from controller!\033[0m\n");
//    }

    // 关节名称
    for (int id = 1; id <= motor_cnt_; id++) {
        joint_names_.push_back(joint_prefix + to_string(id)); // 初始化关节名称
        zero_pos[id-1] = device_.get_zero_position(id); // 获取零点位置
    }

    sub_hmi_ = nh_.subscribe("oil_hmi", 1, &MotorDriver::hmi_callback, this); // 订阅上位机消息
}

// 写控制器寄存器
bool MotorDriver::do_write_operation() {
    if (!init_ready_flag_) return false; // 还未初始化完毕, 不进行写操作

    do_write_flag_ = true;

    if (set_home_ctrl_flag_ && set_all_home()) { // 发送归零指令
        printf("[INFO] Set current pose as home done.\n");
        set_home_ctrl_flag_ = false;
    }

    if (reset_ctrl_flag_ && reset_all_joints()) { // 发送复位指令 NOTE:必须放在写目标位置之前
        printf("[INFO] Reset all joints.\n");
        reset_ctrl_flag_ = false;
    }

    if (power_ctrl_flag_) { // 发送使能/去使能指令
        if (!power_on_flag_) {
            power_on_flag_ = true;
            enable_all_power();
            printf("[INFO] Power on.\n");
        } else {
            power_on_flag_ = false;
            disable_all_power();
            printf("[INFO] Power off.\n");
        }
        power_ctrl_flag_ = false;
    }

//    // 写关节位置
//    for (int id = 1; id <= motor_cnt_; id++) {
//        int offset = OilDevice::offset_goal_position(id); // 控制器寄存器位置偏移
//        // 从Oil寄存器获取目标关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
//        int ret = _write_data(hmi_addr_write_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(id)); // 写目标位置到控制器
//        if (!ret) return false;
//    }

    /// 一次性写入连续可写寄存器, 更省时
    int ret = _write_data(hmi_addr_write_head_, write_reg_len_, (uint16_t *) device_.write_memory());
    if (!ret) return false;

    do_write_flag_ = false;
    return true;
}

// 读控制器寄存器
bool MotorDriver::do_read_operation() {
    do_read_flag_ = true;
    /// 必须读取的寄存器：当前位置、速度、力矩、零点位置
    /// 一次性读取连续可读寄存器, 更省时
    m_master_->modbusReadHoldReg(slaver_, hmi_addr_read_head_, read_reg_len_, (uint16_t *) device_.read_memory());
//    m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, (uint16_t *) device_.memory());

    // 获取当前关节位置
    for (int id = 1; id <= motor_cnt_; id++) {
        curr_pos[id-1] = get_position(id); // 获取当前位置(实际值), 保存到位置缓冲区
    }

    // 获取当前零点位置
    for (int id = 1; id <= motor_cnt_; id++) { // 获取零点位置(寄存器值), 零点在重新设置零点时才会变化, 但此处持续获取确保无误
        zero_pos[id-1] = device_.get_zero_position(id);
    }

    if (show_info_) print_position(5); // 5s打印一次

    // 读关节位置
//    for (int id = 1; id <= motor_cnt_; id++) {
//        uint32_t offset = OilDevice::offset_curr_position(id); // 控制器寄存器位置偏移
//
//        // 从控制器读取当前位置, 写入Oil寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
////        printf("[INFO] offset[%d]: %d addr: %d\n", id, offset, hmi_addr_head_ + offset);
//        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(id));
//        curr_pos[id-1] = get_position(id); // 当前位置保存到位置缓冲区
//    }

//    // 读关节速度
//    for (int id = 1; id <= motor_cnt_; id++) {
//        uint32_t offset = OilDevice::offset_curr_velocity(id); // 控制器寄存器位置偏移
//
//        // 从控制器读取当前速度, 写入Oil寄存器, 在控制器寄存器中长度为1单字, 占两个数据位, 即uint16_t
//        _read_data(hmi_addr_head_ + offset, 1, (uint16_t *)device_.curr_vel_ptr(id));
//        curr_vel[id-1] = device_.get_curr_velocity(id); // 当前位置保存到位置缓冲区
//    }
//    // 读关节力矩
//    for (int id = 1; id <= motor_cnt_; id++) {
//        uint32_t offset = OilDevice::offset_curr_effort(id); // 控制器寄存器位置偏移
//
//        // 从控制器读取当前力矩, 写入Oil寄存器, 在控制器寄存器中长度为1单字, 占两个数据位, 即uint16_t
//        _read_data(hmi_addr_head_ + offset, 1, (uint16_t *)device_.curr_eff_ptr(id));
//        curr_eff[id-1] = device_.get_curr_effort(id); // 当前位置保存到位置缓冲区
//    }

    do_read_flag_ = false;
    return true;
}

// HMI msg 0:使能 1:复位 2:置零 3:急停
void MotorDriver::hmi_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
//    printf("I heard: [ ");
//    for (int i : msg->data) {
//        printf("%d ", i);
//    }
//    printf("]\n");

    // 当前位置设置为初始位置
    if (msg->data[2] == 1) set_home_ctrl_flag_ = true;
    if (msg->data[0] == 1) power_ctrl_flag_ = true;
    if (msg->data[1] == 1) reset_ctrl_flag_ = true;
    if (msg->data[3] == 1) emergency_stop_flag_ = true;
//    if (msg->data[3] == 1) printf("Emergency Stop !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

}

void MotorDriver::print_position(uint8_t time) {
    if (ros::Time::now().toSec() - last_print_time_ > time) { // 单位: s
        last_print_time_ = ros::Time::now().toSec();
        // 获取零位关节角度
        cout << endl << "zero_position     [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", zero_pos[i-1]);
        }
        cout << "] rad [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", R2D(zero_pos[i-1]));
        }
        cout << "] deg" << endl;

        // 获取目标关节角度
        cout << "goal_position     [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", get_goal_position(i));
        }
        cout << "] rad [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", R2D(get_goal_position(i)));
        }
        cout << "] deg" << endl;

        // 获取目标关节角度, 原始值(寄存器值)
        cout << "goal_position_reg [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", device_.get_goal_position(i));
        }
        cout << "] rad [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", R2D(device_.get_goal_position(i)));
        }
        cout << "] deg" << endl;

        // 获取当前关节角度
        cout << "curr_position     [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", get_position(i));
        }
        cout << "] rad [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", R2D(get_position(i)));
        }
        cout << "] deg" << endl;

        // 获取当前关节角度, 原始值(寄存器值)
        cout << "curr_position_reg [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", device_.get_curr_position(i));
        }
        cout << "] rad [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            printf("%0.3f ", R2D(device_.get_curr_position(i)));
        }
        cout << "] deg" << endl;
    }
}

}

