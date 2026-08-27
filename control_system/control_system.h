#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "motor.h"

/* PID 参数结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_ParamTypeDef;

/* 电机目标速度控制结构体 */
typedef struct {
    float target_speed_L;  // 左轮目标转速 (单位: rps 转/秒)
    float target_speed_R;  // 右轮目标转速 (单位: rps 转/秒)
} Motor_SpeedTypeDef;

/* 外部引用的 PID 参数、同步系数与目标速度结构体 (在 main.c 中定义与调整) */
extern PID_ParamTypeDef PID_Motor_L;
extern PID_ParamTypeDef PID_Motor_R;
extern Motor_SpeedTypeDef Car_Speed;
extern float Pos_Sync_Kp;    // 左右轮总里程绝对位置锁步纠偏系数

extern int L_coder, R_coder;
extern long Total_Pulse_L, Total_Pulse_R; // 左右轮累计脉冲总数
extern int Motor_A, Motor_B;
extern int OverFlowTime;

void PID_Reset(void);
void Distance_Reset(void);
long Get_Average_Distance_Pulse(void);
int Incremental_PI_A(int Encoders_A, int Target_A);
int Incremental_PI_B(int Encoders_B, int Target_B);
int Rs_To_CPR(float rads);
void System_Control(void);

#endif
