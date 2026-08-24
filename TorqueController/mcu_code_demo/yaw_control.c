#include <stdint.h>
#include <math.h>
#define M_PI 3.14159265358979323846

uint16_t original_yaw_angle;      // 假设这是 电机原始位置报文 0~8191
int16_t original_yaw_velocity;    // 假设这是 电机原始速度报文 rpm

int32_t yaw_turns = 0;                      // 用于记录yaw轴电机转动圈数
float last_yaw_angle_in_one_turn = 0.0f;    // 辅助圈数计算的变量
uint8_t yaw_turns_inited = 0;               // 辅助圈数计算的变量

uint8_t auto_aim_switch;                // 电控的 自瞄开关
uint8_t auto_aim_enable;                // 上位机传来的 自瞄开关
double yaw_target_angle;                // 上位机传来的
float yaw_target_velocity;              // 上位机传来的
float yaw_torque;                       // 上位机传来的
uint8_t yaw_torque_only_mode;           // 上位机传来的



void yaw_contrl() {
    // 不管是自瞄还是手动模式都要始终处理的部分
    float yaw_angle_in_one_turn = ((float)original_yaw_angle) * (2.0*M_PI / 8192.0f);
    if (yaw_turns_inited) {
        float distance_to_next_turn = fabsf(last_yaw_angle_in_one_turn - 2.0*M_PI - yaw_angle_in_one_turn);
        float distance_to_this_turn = fabsf(last_yaw_angle_in_one_turn - yaw_angle_in_one_turn);
        float distance_to_last_turn = fabsf(last_yaw_angle_in_one_turn + 2.0*M_PI - yaw_angle_in_one_turn);
        if ((distance_to_next_turn < distance_to_this_turn) && (distance_to_next_turn < distance_to_last_turn)) {
            yaw_turns += 1;
        } else if ((distance_to_last_turn < distance_to_next_turn) && (distance_to_last_turn < distance_to_this_turn)) {
            yaw_turns -= 1;
        }
    } else {
        yaw_turns_inited = 1;
    }
    last_yaw_angle_in_one_turn = yaw_angle_in_one_turn;

    double yaw_angle = (double)yaw_angle_in_one_turn + ((double)yaw_turns * 2.0*M_PI); // 多圈连续化
    // 作为发给上位机的 yaw_angle
    float yaw_omega = (float)original_yaw_velocity * M_PI / 30.0; // rpm 转 rad/s
    // 作为发给上位机的 yaw_omega

    if (auto_aim_switch && auto_aim_enable) {
        // 自瞄模式处理流程


        float yaw_control_torque;
        if (yaw_torque_only_mode) { // 两种控制模式
            yaw_control_torque = yaw_torque;
        } else {
            float kp = 0.1f;
            float kd = 0.1f; // 这两个pid参数需要调参

            yaw_control_torque = kp * (yaw_target_angle - yaw_angle) + kd * (yaw_target_velocity - yaw_omega) + yaw_torque;
        }
        if (yaw_control_torque > 1.0f) { // 限位
            yaw_control_torque = 1.0f;
        } else if (yaw_control_torque < -1.0f) {
            yaw_control_torque = -1.0f;
        }

        int16_t yaw_control_torque_to_send = (int16_t)(16384.0f * yaw_control_torque);
        if (yaw_control_torque_to_send > 16384) { // 再次限位
            yaw_control_torque_to_send = 16384;
        } else if (yaw_control_torque_to_send < -16384) {
            yaw_control_torque_to_send = -16384;
        }

        yaw_control_torque_to_send; // 作为发送给电机的电流


    } else {
        // 正常手动操控模式处理流程
        // ...
    }
}