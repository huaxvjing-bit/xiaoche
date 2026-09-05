#ifndef __LINE_TRACKING_H
#define __LINE_TRACKING_H

#include "stm32f10x.h"

/* 主状态：赛道路线阶段 */
typedef enum {
    TRACK_INIT_CRUISE = 0,  /* 初始直行巡航（寻找起点横线，未记述阶段） */
    TRACK_START_PASS,       /* 起点双横线穿越与判定 */
    TRACK_STRAIGHT_1,       /* 起点后的直道，等待实际第1岔口 */
    TRACK_FORK1_LEFT,       /* 原第2岔口：现在的第1岔口，左转 */
    TRACK_CURVE,            /* 两个实际岔口之间的弯道 */
    TRACK_FORK2_RIGHT,      /* 原第3岔口：现在的第2岔口，右转 */
    TRACK_FINAL_STRAIGHT,   /* 终点前直道 */
    TRACK_FINISH_PASS,      /* 终点横线通过 */
    TRACK_STOP_LOCKED       /* 停机锁定 */
} TrackState_t;

/* 子状态：当前事件处理过程 */
typedef enum {
    SUB_NONE = 0,
    SUB_START_LINE1,        /* 起点第一横线上 */
    SUB_START_GAP,          /* 起点双横线间隙 */
    SUB_START_LINE2,        /* 起点第二横线上 */
    SUB_GUIDANCE_ACTIVE,    /* 岔路导向执行中 */
    SUB_LINE_LOST_SEARCH,   /* 丢线搜索 */
    SUB_FINISH_FIRST_LINE,  /* 终点第一横线 */
    SUB_FINISH_GAP,         /* 终点横线间隙 */
    SUB_BRAKING             /* 制动 */
} TrackSubState_t;

void Line_Tracking_Init(void);
void Line_Tracking_Task(void);

#endif
