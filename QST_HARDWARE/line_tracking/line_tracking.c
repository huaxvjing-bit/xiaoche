#include "line_tracking.h"
#include "motor.h"
#include "cliff_sensor.h"
#include <stdio.h>

/*
 * 两传感器、单优先级循迹控制：
 *   双黑 > 单灯黑 > 正常前进
 * 每次任务周期为10ms，由USER/main.c调用Line_Tracking_Task()。
 */
#define TRACK_FORWARD_PWM       3600
#define TRACK_SHIFT_PWM         3200
#define TRACK_SHIFT_CYCLES      5U       /* 一次偏移50ms */
#define FORK_TURN_PWM           3200
#define FORK1_TURN_CYCLES       30U      /* 第1岔口：0.30s */
#define FORK2_TURN_CYCLES       25U      /* 第2岔口：0.25s */
#define FORK1_EXIT_FORWARD_CYCLES 10U   /* 第1岔口出岔保护100ms */
#define FORK2_EXIT_FORWARD_CYCLES 30U   /* 第2岔口出岔保护300ms，抑制反向抢控 */
#define SENSOR_CONFIRM_CYCLES   1U       /* 立即响应，不延迟单灯黑 */

#define SENSOR_LEFT_BLACK       1U
#define SENSOR_RIGHT_BLACK      2U
#define SENSOR_BOTH_BLACK       3U

/* 当前循迹动作 */
typedef enum {
    ACTION_NONE = 0,
    ACTION_STOP,
    ACTION_SHIFT
} TrackAction_t;

/* 岔口动作 */
typedef enum {
    FORK_NONE = 0,
    FORK_TURN,
    FORK_EXIT_FORWARD
} ForkAction_t;

static TrackState_t s_state = TRACK_STRAIGHT_1;
static TrackSubState_t s_sub = SUB_NONE;
static u8 s_stopped = 0U;
static u8 s_fork_count = 0U;
static u8 s_double_black_lock = 0U;
static u8 s_action_side = 0U;       /* 1=左侧黑/左移，2=右侧黑/右移 */
static TrackAction_t s_action = ACTION_NONE;
static u8 s_action_cycles = 0U;
static ForkAction_t s_fork_action = FORK_NONE;
static u8 s_fork_dir = 0U;          /* 1=左转，2=右转 */
static u8 s_fork_cycles = 0U;
static u8 s_fork_exit_cycles = 0U;
static u8 s_log_timer = 0U;
static u8 s_last_pattern = 0U;

static u8 Sensor_Pattern(u8 left, u8 right)
{
    return (u8)((left ? 1U : 0U) | (right ? 2U : 0U));
}

/* Set_Pwm使用有符号PWM：正值=前进，负值=反向，0=停止。 */
static int Limit_Pwm(int value)
{
    if (value > 6000) return 6000;
    if (value < -6000) return -6000;
    return value;
}

static void Logical_Pwm(int left, int right)
{
    Set_Pwm(Limit_Pwm(left), Limit_Pwm(right));
}

static void Emergency_Stop(void)
{
    Set_Pwm(0, 0);
}

static void Normal_Forward(void)
{
    Logical_Pwm(TRACK_FORWARD_PWM, TRACK_FORWARD_PWM);
}

static void Start_Single_Shift(u8 side)
{
    s_action_side = side;
    s_action = ACTION_STOP;
    s_action_cycles = 0U;
    Emergency_Stop();
}

static void Run_Single_Shift(void)
{
    if (s_action == ACTION_STOP) {
        /* 停顿一个完整10ms周期，给双黑检测和当前动作中止让出最高优先级。 */
        Emergency_Stop();
        s_action = ACTION_SHIFT;
        s_action_cycles = 0U;
        return;
    }

    if (s_action != ACTION_SHIFT) return;

    /* 实车方向校正：左灯黑时向左转，右灯黑时向右转。 */
    if (s_action_side == 1U) {
        Logical_Pwm(TRACK_SHIFT_PWM, -TRACK_SHIFT_PWM);
    } else {
        Logical_Pwm(-TRACK_SHIFT_PWM, TRACK_SHIFT_PWM);
    }
    s_action_cycles++;
    if (s_action_cycles >= TRACK_SHIFT_CYCLES) {
        s_action = ACTION_NONE;
        s_action_cycles = 0U;
        s_action_side = 0U;
    }
}

static void Start_Fork_Turn(u8 dir)
{
    s_fork_dir = dir;
    s_fork_action = FORK_TURN;
    s_fork_cycles = 0U;
    s_action = ACTION_NONE;
    s_action_side = 0U;
    Emergency_Stop();
    printf("[FORK] emergency stop, turn=%s\r\n", dir == 1U ? "LEFT" : "RIGHT");
}

static u8 Run_Fork_Turn(void)
{
    if (s_fork_action == FORK_EXIT_FORWARD) {
        /* 出岔保护：保持转向后的方向直行，禁止单灯动作反向抢控制权。 */
        Normal_Forward();
        s_fork_exit_cycles++;
        if (s_fork_exit_cycles >= ((s_fork_count == 1U) ?
                                  FORK1_EXIT_FORWARD_CYCLES : FORK2_EXIT_FORWARD_CYCLES)) {
            s_fork_action = FORK_NONE;
            s_fork_exit_cycles = 0U;
            printf("[FORK] exit protection complete\r\n");
        }
        return 1U;
    }

    if (s_fork_action != FORK_TURN) return 0U;

    /* 原地转向：一轮前进、另一轮反向，绝不加入两轮同向前进。 */
    if (s_fork_dir == 1U) {
        Logical_Pwm(FORK_TURN_PWM, -FORK_TURN_PWM);
    } else {
        Logical_Pwm(-FORK_TURN_PWM, FORK_TURN_PWM);
    }
    s_fork_cycles++;
    if (s_fork_cycles >= ((s_fork_count == 1U) ?
                          FORK1_TURN_CYCLES : FORK2_TURN_CYCLES)) {
        if (s_fork_count == 1U) {
            s_state = TRACK_CURVE;
            s_sub = SUB_NONE;
        } else {
            s_state = TRACK_FINAL_STRAIGHT;
            s_sub = SUB_NONE;
        }
        s_fork_action = FORK_EXIT_FORWARD;
        s_fork_cycles = 0U;
        s_fork_exit_cycles = 0U;
        Normal_Forward();
        printf("[FORK] turn complete, exit protection\r\n");
    }
    return 1U;
}

static void Normal_Tracking(u8 pattern)
{
    if (pattern == SENSOR_LEFT_BLACK) {
        if (s_action == ACTION_NONE) Start_Single_Shift(1U);
        return;
    }
    if (pattern == SENSOR_RIGHT_BLACK) {
        if (s_action == ACTION_NONE) Start_Single_Shift(2U);
        return;
    }
    if (pattern == 0U) {
        Normal_Forward();
        return;
    }
    /* 普通道路双黑只保持前进；路线双黑在主任务中优先处理。 */
    Normal_Forward();
}

void Line_Tracking_Init(void)
{
    Cliff_Sensor_Init();
    s_state = TRACK_STRAIGHT_1;
    s_sub = SUB_NONE;
    s_stopped = 0U;
    s_fork_count = 0U;
    s_double_black_lock = 0U;
    s_action_side = 0U;
    s_action = ACTION_NONE;
    s_action_cycles = 0U;
    s_fork_action = FORK_NONE;
    s_fork_dir = 0U;
    s_fork_cycles = 0U;
    s_fork_exit_cycles = 0U;
    s_log_timer = 0U;
    s_last_pattern = 0U;
    Emergency_Stop();
}

void Line_Tracking_Task(void)
{
    u8 left;
    u8 right;
    u8 pattern;

    if (s_stopped != 0U) {
        Emergency_Stop();
        return;
    }

    left = GPIO_ReadInputDataBit(CLIFF_PORT, CLIFF_PIN_L) ? 1U : 0U;
    right = GPIO_ReadInputDataBit(CLIFF_PORT, CLIFF_PIN_R) ? 1U : 0U;
    pattern = Sensor_Pattern(left, right);
    s_last_pattern = pattern;

    /* 岔口转向及出岔保护期间保持锁定，防止同一岔口重复触发反向转动。 */
    if (s_fork_action == FORK_NONE && pattern != SENSOR_BOTH_BLACK) {
        s_double_black_lock = 0U;
    }

    /* 岔口动作是全局最高优先级：转向和出岔保护期间均不可被任何传感器动作打断。 */
    if (s_fork_action != FORK_NONE) {
        Run_Fork_Turn();
        return;
    }

    /* 双黑是全局最高优先级：中止正在进行的单灯停顿/偏移动作。 */
    if (pattern == SENSOR_BOTH_BLACK && s_double_black_lock == 0U) {
        s_double_black_lock = 1U;
        s_action = ACTION_NONE;
        s_action_side = 0U;
        Emergency_Stop();
        if (s_fork_count < 2U) {
            s_fork_count++;
            Start_Fork_Turn(s_fork_count == 1U ? 1U : 2U);
        } else {
            Normal_Forward();
        }
        return;
    }

    /* 单灯纠偏动作期间不被普通前进覆盖；若下一次仍为该侧黑，重新偏移。 */
    if (s_action != ACTION_NONE) {
        if (pattern == SENSOR_BOTH_BLACK) {
            s_action = ACTION_NONE;
            s_action_side = 0U;
            Emergency_Stop();
            return;
        }
        if (pattern == s_action_side || pattern == 0U) {
            Run_Single_Shift();
            return;
        }
        s_action = ACTION_NONE;
        s_action_side = 0U;
    }

    Normal_Tracking(pattern);

    s_log_timer++;
    if (s_log_timer >= 20U) {
        s_log_timer = 0U;
        printf("TRACK ST=%d SUB=%d FORK=%d PAT=%d ACT=%d\r\n",
               s_state, s_sub, s_fork_count, s_last_pattern, s_action);
    }
}
