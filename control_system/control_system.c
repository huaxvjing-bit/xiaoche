#include "control_system.h"

/* 电机单周期编码器测量值 左A 右B */
int L_coder = 0, R_coder = 0;

/* 电机累计脉冲数（用于精准1米距离控制与绝对位置锁步） */
long Total_Pulse_L = 0;
long Total_Pulse_R = 0;

int Motor_A = 0, Motor_B = 0;   // 电机PWM变量
int OverFlowTime = 100;

/* PID 控制器内部状态变量 */
static float Integral_A = 0.0f;
static float Error_prev_A = 0.0f;

static float Integral_B = 0.0f;
static float Error_prev_B = 0.0f;

/* 完全重置 PID 状态与历史误差 */
void PID_Reset(void)
{
    Integral_A = 0.0f;
    Error_prev_A = 0.0f;

    Integral_B = 0.0f;
    Error_prev_B = 0.0f;

    Motor_A = 0;
    Motor_B = 0;
    Set_Pwm(0, 0);
}

/* 重置累计里程脉冲 */
void Distance_Reset(void)
{
    Total_Pulse_L = 0;
    Total_Pulse_R = 0;
}

/* 获取当前双轮平均累计脉冲绝对值 */
long Get_Average_Distance_Pulse(void)
{
    long abs_L = Total_Pulse_L >= 0 ? Total_Pulse_L : -Total_Pulse_L;
    long abs_R = Total_Pulse_R >= 0 ? Total_Pulse_R : -Total_Pulse_R;
    return (abs_L + abs_R) / 2;
}

/**************************************************************************************
函数功能：速度闭环 PI 控制器 (左电机A)
**************************************************************************************/
int Incremental_PI_A(int Encoders_A, int Target_A)
{
    float Error_A;
    float Output_A;

    if (Target_A == 0)
    {
        Integral_A = 0.0f;
        Error_prev_A = 0.0f;
        return 0;
    }

    Error_A = (float)(Target_A - Encoders_A);

    Integral_A += Error_A;
    if (Integral_A > 3000.0f) Integral_A = 3000.0f;
    else if (Integral_A < -3000.0f) Integral_A = -3000.0f;

    Output_A = PID_Motor_L.Kp * Error_A + PID_Motor_L.Ki * Integral_A + PID_Motor_L.Kd * (Error_A - Error_prev_A);

    Error_prev_A = Error_A;

    if (Output_A > 7199.0f) Output_A = 7199.0f;
    else if (Output_A < -7199.0f) Output_A = -7199.0f;

    return (int)Output_A;
}

/**************************************************************************************
函数功能：速度闭环 PI 控制器 (右电机B)
**************************************************************************************/
int Incremental_PI_B(int Encoders_B, int Target_B)
{
    float Error_B;
    float Output_B;

    if (Target_B == 0)
    {
        Integral_B = 0.0f;
        Error_prev_B = 0.0f;
        return 0;
    }

    Error_B = (float)(Target_B - Encoders_B);

    Integral_B += Error_B;
    if (Integral_B > 3000.0f) Integral_B = 3000.0f;
    else if (Integral_B < -3000.0f) Integral_B = -3000.0f;

    Output_B = PID_Motor_R.Kp * Error_B + PID_Motor_R.Ki * Integral_B + PID_Motor_R.Kd * (Error_B - Error_prev_B);

    Error_prev_B = Error_B;

    if (Output_B > 7199.0f) Output_B = 7199.0f;
    else if (Output_B < -7199.0f) Output_B = -7199.0f;

    return (int)Output_B;
}

/**************************************************************************************
函数功能：转每秒转脉冲数函数
**************************************************************************************/
int Rs_To_CPR(float rads)
{
    int CRP = 0;
    CRP = (int)(rads * ((700 * 4) / (1000 / OverFlowTime)));
    return CRP;
}

/**************************************************************************************
函数功能：系统控制函数 (仿照倒车高灵敏响应机制：自适应正反向锁步对齐算法)
**************************************************************************************/
void System_Control(void)
{
    int Base_TageA = 0;
    int Base_TageB = 0;
    int Real_TageA = 0;
    int Real_TageB = 0;
    float Pos_Diff = 0.0f;
    float Sync_Target_Adj = 0.0f;

    // 1. 读取单周期脉冲并累计总里程脉冲
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    Total_Pulse_L += L_coder;
    Total_Pulse_R += R_coder;

    printf("L: %d, R: %d | Total_L: %ld, Total_R: %ld\r\n",
           L_coder, R_coder, Total_Pulse_L, Total_Pulse_R);

    // 2. 根据目标速度计算基础目标脉冲数
    Base_TageA = Rs_To_CPR(Car_Speed.target_speed_L);
    Base_TageB = Rs_To_CPR(Car_Speed.target_speed_R);

    Real_TageA = Base_TageA;
    Real_TageB = Base_TageB;

    // 3. 仿照倒车高灵敏响应机制：自适应正反向锁步纠偏
    if (Base_TageA != 0 && Base_TageB != 0 && (Car_Speed.target_speed_L == Car_Speed.target_speed_R))
    {
        Pos_Diff = (float)(Total_Pulse_L - Total_Pulse_R);
        Sync_Target_Adj = Pos_Sync_Kp * Pos_Diff;

        // 柔性限幅：单周期修正量限制在 +-40 脉冲以内
        if (Sync_Target_Adj > 40.0f) Sync_Target_Adj = 40.0f;
        else if (Sync_Target_Adj < -40.0f) Sync_Target_Adj = -40.0f;

        if (Base_TageA > 0) // 前进状态：强化纠偏响应 (1.35f)
        {
            Real_TageA = Base_TageA - (int)(Sync_Target_Adj * 1.4f);
            Real_TageB = Base_TageB + (int)(Sync_Target_Adj * 1.4f);
        }
        else // 后退状态：保持原生对齐响应
        {
            Real_TageA = Base_TageA - (int)Sync_Target_Adj;
            Real_TageB = Base_TageB + (int)Sync_Target_Adj;
        }
    }

    // 4. 速度闭环控制计算左右电机最终 PWM
    Motor_A = Incremental_PI_A(L_coder, Real_TageA);
    Motor_B = Incremental_PI_B(R_coder, Real_TageB);

    printf("TageA: %d, TageB: %d | Motor_A: %d, Motor_B: %d\r\n",
           Real_TageA, Real_TageB, Motor_A, Motor_B);

    // 5. 设置电机输出
    Set_Pwm(Motor_A, Motor_B);
}
