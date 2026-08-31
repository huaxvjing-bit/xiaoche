#include "control_system.h"

/* 实时编码器读数 */
int L_coder = 0, R_coder = 0;

/* 双轮累积脉冲计数 (用于定距与定角闭环) */
long Total_Pulse_L = 0;
long Total_Pulse_R = 0;

int Motor_A = 0, Motor_B = 0;   // 电机PWM输出
int OverFlowTime = 100;

/* 电机静摩擦死区前馈电压 (保证起步瞬间爆发足够扭矩克服摩擦) */
#define MOTOR_DEADZONE_PWM     1800.0f

/* PID 控制器内部状态变量 */
static float Integral_A = 0.0f;
static float Error_prev_A = 0.0f;

static float Integral_B = 0.0f;
static float Error_prev_B = 0.0f;

/* 完全复位 PID 状态与历史误差 */
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

/* 复位累积脉冲数 */
void Distance_Reset(void)
{
    Total_Pulse_L = 0;
    Total_Pulse_R = 0;
}

/* 获取当前双轮绝对值平均累积脉冲数 */
long Get_Average_Distance_Pulse(void)
{
    long abs_L = Total_Pulse_L >= 0 ? Total_Pulse_L : -Total_Pulse_L;
    long abs_R = Total_Pulse_R >= 0 ? Total_Pulse_R : -Total_Pulse_R;
    return (abs_L + abs_R) / 2;
}

/**************************************************************************************
* 函数功能：带前馈死区补偿的高响应速度闭环 PI 控制器 (左电机A)
***************************************************************************************/
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

    // 前馈死区补偿 + PID 闭环调节
    Output_A = PID_Motor_L.Kp * Error_A + PID_Motor_L.Ki * Integral_A + PID_Motor_L.Kd * (Error_A - Error_prev_A);

    if (Target_A > 0)
    {
        Output_A += MOTOR_DEADZONE_PWM;
    }
    else if (Target_A < 0)
    {
        Output_A -= MOTOR_DEADZONE_PWM;
    }

    Error_prev_A = Error_A;

    if (Output_A > 7199.0f) Output_A = 7199.0f;
    else if (Output_A < -7199.0f) Output_A = -7199.0f;

    return (int)Output_A;
}

/**************************************************************************************
* 函数功能：带前馈死区补偿的高响应速度闭环 PI 控制器 (右电机B)
***************************************************************************************/
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

    // 前馈死区补偿 + PID 闭环调节
    Output_B = PID_Motor_R.Kp * Error_B + PID_Motor_R.Ki * Integral_B + PID_Motor_R.Kd * (Error_B - Error_prev_B);

    if (Target_B > 0)
    {
        Output_B += MOTOR_DEADZONE_PWM;
    }
    else if (Target_B < 0)
    {
        Output_B -= MOTOR_DEADZONE_PWM;
    }

    Error_prev_B = Error_B;

    if (Output_B > 7199.0f) Output_B = 7199.0f;
    else if (Output_B < -7199.0f) Output_B = -7199.0f;

    return (int)Output_B;
}

/**************************************************************************************
* 函数功能：转/秒 (rps) 转换为单采样周期的脉冲数 CPR
***************************************************************************************/
int Rs_To_CPR(float rads)
{
    int CRP = 0;
    CRP = (int)(rads * ((700 * 4) / (1000 / OverFlowTime)));
    return CRP;
}

/**************************************************************************************
* 函数功能：系统控制核心周期函数 (100ms周期调用, 零阻塞、高响应)
***************************************************************************************/
void System_Control(void)
{
    int Base_TageA = 0;
    int Base_TageB = 0;
    int Real_TageA = 0;
    int Real_TageB = 0;
    float Pos_Diff = 0.0f;
    float Sync_Target_Adj = 0.0f;

    // 1. 读取编码器脉冲并累加总脉冲
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    Total_Pulse_L += L_coder;
    Total_Pulse_R += R_coder;

    // 2. 将目标转速转换为目标脉冲
    Base_TageA = Rs_To_CPR(Car_Speed.target_speed_L);
    Base_TageB = Rs_To_CPR(Car_Speed.target_speed_R);

    Real_TageA = Base_TageA;
    Real_TageB = Base_TageB;

    // 3. 直线行驶状态下的左右轮位置同步纠偏
    if (Base_TageA != 0 && Base_TageB != 0 && (Car_Speed.target_speed_L == Car_Speed.target_speed_R))
    {
        Pos_Diff = (float)(Total_Pulse_L - Total_Pulse_R);
        Sync_Target_Adj = Pos_Sync_Kp * Pos_Diff;

        // 纠偏量限幅
        if (Sync_Target_Adj > 40.0f) Sync_Target_Adj = 40.0f;
        else if (Sync_Target_Adj < -40.0f) Sync_Target_Adj = -40.0f;

        if (Base_TageA > 0) // 前进状态强化纠偏
        {
            Real_TageA = Base_TageA - (int)(Sync_Target_Adj * 1.4f);
            Real_TageB = Base_TageB + (int)(Sync_Target_Adj * 1.4f);
        }
        else // 后退状态
        {
            Real_TageA = Base_TageA - (int)Sync_Target_Adj;
            Real_TageB = Base_TageB + (int)Sync_Target_Adj;
        }
    }

    // 4. 速度闭环控制计算左右电机输出 PWM
    Motor_A = Incremental_PI_A(L_coder, Real_TageA);
    Motor_B = Incremental_PI_B(R_coder, Real_TageB);

    // 5. 设置电机输出
    Set_Pwm(Motor_A, Motor_B);
}

/**************************************************************************************
* 函数功能：停车与阻尼制动
***************************************************************************************/
void Car_Brake(u16 brake_duration_ms)
{
    u16 i;
    u16 cycles = brake_duration_ms / 100;
    if (cycles == 0) cycles = 1;

    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    PID_Reset();

    for (i = 0; i < cycles; i++)
    {
        System_Control();
        delay_ms(100);
    }
    Set_Pwm(0, 0);
}

/**************************************************************************************
* 函数功能：闭环定距前进
***************************************************************************************/
void Move_Forward(long pulse, float speed)
{
    Read_Encoder(2);
    Read_Encoder(3);
    Distance_Reset();
    PID_Reset();

    Car_Speed.target_speed_L = speed > 0 ? speed : -speed;
    Car_Speed.target_speed_R = speed > 0 ? speed : -speed;

    while (Get_Average_Distance_Pulse() < pulse)
    {
        System_Control();
        delay_ms(100);
    }
}

/**************************************************************************************
* 函数功能：闭环定距后退
***************************************************************************************/
void Move_Backward(long pulse, float speed)
{
    Read_Encoder(2);
    Read_Encoder(3);
    Distance_Reset();
    PID_Reset();

    Car_Speed.target_speed_L = speed > 0 ? -speed : speed;
    Car_Speed.target_speed_R = speed > 0 ? -speed : speed;

    while (Get_Average_Distance_Pulse() < pulse)
    {
        System_Control();
        delay_ms(100);
    }
}

/**************************************************************************************
* 函数功能：闭环定角原地左转
***************************************************************************************/
void Turn_Left(long pulse, float speed)
{
    float sp = speed > 0 ? speed : -speed;
    Read_Encoder(2);
    Read_Encoder(3);
    Distance_Reset();
    PID_Reset();

    Car_Speed.target_speed_L = -sp;
    Car_Speed.target_speed_R =  sp;

    while (Get_Average_Distance_Pulse() < pulse)
    {
        System_Control();
        delay_ms(100);
    }
}

/**************************************************************************************
* 函数功能：闭环定角原地右转
***************************************************************************************/
void Turn_Right(long pulse, float speed)
{
    float sp = speed > 0 ? speed : -speed;
    Read_Encoder(2);
    Read_Encoder(3);
    Distance_Reset();
    PID_Reset();

    Car_Speed.target_speed_L =  sp;
    Car_Speed.target_speed_R = -sp;

    while (Get_Average_Distance_Pulse() < pulse)
    {
        System_Control();
        delay_ms(100);
    }
}

