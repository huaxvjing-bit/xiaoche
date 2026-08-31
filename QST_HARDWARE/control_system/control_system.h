#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "motor.h"
#include "delay.h"

/* Speed Base Unit (rps) */
#define BASE_SPEED_UNIT        0.70f

/* PID Structure */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_ParamTypeDef;

/* Target Speed Structure */
typedef struct {
    float target_speed_L;
    float target_speed_R;
} Motor_SpeedTypeDef;

/* Externs */
extern PID_ParamTypeDef PID_Motor_L;
extern PID_ParamTypeDef PID_Motor_R;
extern Motor_SpeedTypeDef Car_Speed;
extern float Pos_Sync_Kp;

extern int L_coder, R_coder;
extern long Total_Pulse_L, Total_Pulse_R;
extern int Motor_A, Motor_B;
extern int OverFlowTime;

/* Core Functions */
void PID_Reset(void);
void Distance_Reset(void);
long Get_Average_Distance_Pulse(void);
int Incremental_PI_A(int Encoders_A, int Target_A);
int Incremental_PI_B(int Encoders_B, int Target_B);
int Rs_To_CPR(float rads);
void System_Control(void);

/* Motion Command Parser with Vector Superposition */
void Execute_Motion_Command(const char *cmd, u8 len);

/* Motion Actions */
void Move_Forward(long pulse, float speed);
void Move_Backward(long pulse, float speed);
void Turn_Left(long pulse, float speed);
void Turn_Right(long pulse, float speed);
void Car_Brake(u16 brake_duration_ms);

#endif
