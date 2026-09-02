#include "control_system.h"

/* Encoders */
int L_coder = 0, R_coder = 0;

/* Total Odometry Pulses */
long Total_Pulse_L = 0;
long Total_Pulse_R = 0;

/* Independent Straight Segment Pulses (Zero historical memory across turns) */
static long Straight_Pulse_L = 0;
static long Straight_Pulse_R = 0;

/* State Machine Transition Tracking */
static float Prev_Target_L = 0.0f;
static float Prev_Target_R = 0.0f;

int Motor_A = 0, Motor_B = 0;
int OverFlowTime = 100;

/* Motor Static Friction Deadzone */
#define MOTOR_DEADZONE_PWM     1800.0f

/* PID Internal States */
static float Integral_A = 0.0f;
static float Error_prev_A = 0.0f;

static float Integral_B = 0.0f;
static float Error_prev_B = 0.0f;

/* Reset PID */
void PID_Reset(void)
{
    Integral_A = 0.0f;
    Error_prev_A = 0.0f;

    Integral_B = 0.0f;
    Error_prev_B = 0.0f;

    Straight_Pulse_L = 0;
    Straight_Pulse_R = 0;

    Motor_A = 0;
    Motor_B = 0;
    Set_Pwm(0, 0);
}

/* Reset Pulses */
void Distance_Reset(void)
{
    Total_Pulse_L = 0;
    Total_Pulse_R = 0;
    Straight_Pulse_L = 0;
    Straight_Pulse_R = 0;
}

/* Get Average Distance */
long Get_Average_Distance_Pulse(void)
{
    long abs_L = Total_Pulse_L >= 0 ? Total_Pulse_L : -Total_Pulse_L;
    long abs_R = Total_Pulse_R >= 0 ? Total_Pulse_R : -Total_Pulse_R;
    return (abs_L + abs_R) / 2;
}

/* Left Motor PI */
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

    if (Target_A > 0) Output_A += MOTOR_DEADZONE_PWM;
    else if (Target_A < 0) Output_A -= MOTOR_DEADZONE_PWM;

    Error_prev_A = Error_A;

    if (Output_A > 7199.0f) Output_A = 7199.0f;
    else if (Output_A < -7199.0f) Output_A = -7199.0f;

    return (int)Output_A;
}

/* Right Motor PI */
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

    if (Target_B > 0) Output_B += MOTOR_DEADZONE_PWM;
    else if (Target_B < 0) Output_B -= MOTOR_DEADZONE_PWM;

    Error_prev_B = Error_B;

    if (Output_B > 7199.0f) Output_B = 7199.0f;
    else if (Output_B < -7199.0f) Output_B = -7199.0f;

    return (int)Output_B;
}

/* Speed to CPR */
int Rs_To_CPR(float rads)
{
    int CRP = (int)(rads * ((700 * 4) / (1000 / OverFlowTime)));
    return CRP;
}

/* Second-Gen Motion Command Vector Parser */
void Execute_Motion_Command(const char *cmd, u8 len)
{
    u8 i;
    int fwd_count = 0;
    int back_count = 0;
    int left_count = 0;
    int right_count = 0;
    int net_longitudinal = 0;
    int net_lateral = 0;
    float sum_L = 0.0f;
    float sum_R = 0.0f;

    if (len == 0 || cmd == 0) return;

    /* Emergency Stop if '0' or space is present */
    for (i = 0; i < len; i++)
    {
        if (cmd[i] == '0' || cmd[i] == 'x' || cmd[i] == 'X' || cmd[i] == 'S' || cmd[i] == ' ')
        {
            Car_Speed.target_speed_L = 0.0f;
            Car_Speed.target_speed_R = 0.0f;
            PID_Reset();
            return;
        }
    }

    /* Extract and count direction key events */
    for (i = 0; i < len; i++)
    {
        switch (cmd[i])
        {
            case '1': /* Forward */
            case 'w':
            case 'W':
            case 'F':
                fwd_count++;
                break;

            case '2': /* Backward */
            case 's':
            case 'B':
            case 'b':
                back_count++;
                break;

            case '3': /* Left Turn */
            case 'a':
            case 'A':
            case 'L':
            case 'l':
                left_count++;
                break;

            case '4': /* Right Turn */
            case 'd':
            case 'D':
            case 'R':
            case 'r':
                right_count++;
                break;

            default:
                break;
        }
    }

    net_longitudinal = fwd_count - back_count;
    net_lateral = left_count - right_count;

    /* Calculate directional speeds with kinematic consistency */
    if (net_longitudinal > 0)
    {
        /* 前进运动: 1=单倍速前进, 11=双倍速前进, 13=左前, 14=右前 */
        if (net_lateral > 0)       /* 左前 (如 13: 左轮=1, 右轮=1+1=2) */
        {
            sum_L = (float)net_longitudinal;
            sum_R = (float)net_longitudinal + (float)net_lateral;
        }
        else if (net_lateral < 0)  /* 右前 (如 14: 左轮=1+1=2, 右轮=1) */
        {
            sum_L = (float)net_longitudinal + (float)(-net_lateral);
            sum_R = (float)net_longitudinal;
        }
        else                       /* 直行前进 (如 1: 左轮=1,右轮=1; 11: 左轮=2,右轮=2) */
        {
            sum_L = (float)net_longitudinal;
            sum_R = (float)net_longitudinal;
        }
    }
    else if (net_longitudinal < 0)
    {
        /* 后退运动: 2=单倍速后退, 22=双倍速后退, 23=左后, 24=右后 */
        int abs_back = -net_longitudinal;
        if (net_lateral > 0)       /* 左后 (如 23: 左轮=-1, 右轮=-1-1=-2, 与13弧线对称) */
        {
            sum_L = -(float)abs_back;
            sum_R = -(float)(abs_back + net_lateral);
        }
        else if (net_lateral < 0)  /* 右后 (如 24: 左轮=-1-1=-2, 右轮=-1, 与14弧线对称) */
        {
            sum_L = -(float)(abs_back + (-net_lateral));
            sum_R = -(float)abs_back;
        }
        else                       /* 直行后退 (如 2: 左轮=-1,右轮=-1; 22: 左轮=-2,右轮=-2) */
        {
            sum_L = -(float)abs_back;
            sum_R = -(float)abs_back;
        }
    }
    else
    {
        /* 无前后分量时的转向 (如 3: 纯左转, 4: 纯右转) */
        if (net_lateral > 0)       /* 纯左转 */
        {
            sum_L = 0.0f;
            sum_R = (float)net_lateral;
        }
        else if (net_lateral < 0)  /* 纯右转 */
        {
            sum_L = (float)(-net_lateral);
            sum_R = 0.0f;
        }
        else
        {
            sum_L = 0.0f;
            sum_R = 0.0f;
        }
    }

    /* Final Speed Calculations */
    Car_Speed.target_speed_L = sum_L * BASE_SPEED_UNIT;
    Car_Speed.target_speed_R = sum_R * BASE_SPEED_UNIT;

    /* Zero speed cancellation */
    if (sum_L == 0.0f && sum_R == 0.0f)
    {
        PID_Reset();
    }
}

/* System Control Cycle */
void System_Control(void)
{
    int Base_TageA = 0;
    int Base_TageB = 0;
    int Real_TageA = 0;
    int Real_TageB = 0;
    float Pos_Diff = 0.0f;
    float Sync_Target_Adj = 0.0f;

    /* 1. Read Encoders */
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    Total_Pulse_L += L_coder;
    Total_Pulse_R += R_coder;

    /* 2. Transition Detection: Clear straight baseline on target change */
    if (Car_Speed.target_speed_L != Prev_Target_L || Car_Speed.target_speed_R != Prev_Target_R)
    {
        Prev_Target_L = Car_Speed.target_speed_L;
        Prev_Target_R = Car_Speed.target_speed_R;
        Straight_Pulse_L = 0;
        Straight_Pulse_R = 0;

        if (Car_Speed.target_speed_L == 0.0f) { Integral_A = 0.0f; Error_prev_A = 0.0f; }
        if (Car_Speed.target_speed_R == 0.0f) { Integral_B = 0.0f; Error_prev_B = 0.0f; }
    }

    /* 3. Convert target rps to CPR pulses */
    Base_TageA = Rs_To_CPR(Car_Speed.target_speed_L);
    Base_TageB = Rs_To_CPR(Car_Speed.target_speed_R);

    Real_TageA = Base_TageA;
    Real_TageB = Base_TageB;

    /* 4. Pure Straight Line Position Sync (Zero memory across turns) */
    if (Base_TageA != 0 && Base_TageB != 0 && (Car_Speed.target_speed_L == Car_Speed.target_speed_R))
    {
        Straight_Pulse_L += L_coder;
        Straight_Pulse_R += R_coder;

        Pos_Diff = (float)(Straight_Pulse_L - Straight_Pulse_R);
        Sync_Target_Adj = Pos_Sync_Kp * Pos_Diff;

        if (Sync_Target_Adj > 30.0f) Sync_Target_Adj = 30.0f;
        else if (Sync_Target_Adj < -30.0f) Sync_Target_Adj = -30.0f;

        Real_TageA = Base_TageA - (int)Sync_Target_Adj;
        Real_TageB = Base_TageB + (int)Sync_Target_Adj;
    }
    else
    {
        Straight_Pulse_L = 0;
        Straight_Pulse_R = 0;
    }

    /* 5. Closed-loop PID */
    Motor_A = Incremental_PI_A(L_coder, Real_TageA);
    Motor_B = Incremental_PI_B(R_coder, Real_TageB);

    /* 6. Output PWM */
    Set_Pwm(Motor_A, Motor_B);
}

/* Brake */
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

/* Action APIs */
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
