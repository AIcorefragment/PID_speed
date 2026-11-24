#ifndef __PID_SPEED_H__
#define __PID_SPEED_H__

/*-------------------------------------------------------------------------------------------------------------------
 * 速度环 PID
 *   输入：反馈转速 rpm
 *   输出：C620 电流计数（-16384 ~ +16384 对应 -20A ~ +20A）
 *   单独速度模式：调用 PID_Speed_Control()，目标转速在 g_target_rpm（Watch 可改）
 *   串级角度模式：由 pid_angle.c 调用 SpeedPID_Ramp/Update，目标转速来自角度环
 *-----------------------------------------------------------------------------------------------------------------*/
typedef struct{
  float kp;           /* 比例：电流计数 / rpm */
  float ki;           /* 积分：电流计数 / (rpm*s) */
  float kd;           /* 微分：一般先给 0 */
  float target;       /* 目标转速 rpm */
  float target_ramp;  /* 斜坡后的目标 */
  float feedback;     /* 反馈转速 rpm */
  float fb_prev;      /* 上一拍反馈，用于微分 */
  float err;          /* 误差 */
  float err_prev;
  float integral;     /* 积分项，单位是电流计数 */
  float out;          /* 输出电流计数 */
  float out_max;      /* 输出限幅，16384 对应 20A */
  float i_max;        /* 积分限幅 */
  float acc;          /* 目标斜坡，rpm/s */
}SpeedPID_t;

/* 速度环实例（定义在 pid_speed.c） */
extern SpeedPID_t g_pid;

/* 单独速度模式的目标转速 rpm */
extern volatile float g_target_rpm;

void SpeedPID_Init(SpeedPID_t *p);
void SpeedPID_Ramp(SpeedPID_t *p, float target, float dt);
float SpeedPID_Update(SpeedPID_t *p, float feedback, float dt);

/* 单独速度模式入口：斜坡 + PID，结果写入 g_pid.out */
void PID_Speed_Control(float speed_rpm, float dt);

#endif
