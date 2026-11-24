#ifndef __PID_ANGLE_H__
#define __PID_ANGLE_H__

#include <stdint.h>

/*-------------------------------------------------------------------------------------------------------------------
 * 角度环 PID（串级外环）
 *   输入：转子角度 deg（0~360）、角速度 deg/s、反馈转速 rpm
 *   输出：速度环的目标转速 rpm（内部调用 pid_speed 的速度环）
 *   单独角度模式：调用 PID_Angle_Control()，目标角度在 g_target_deg（Watch 可改）
 *   注意：这里是转子单圈角度，输出轴角度 = 转子角度 / 减速比
 *-----------------------------------------------------------------------------------------------------------------*/
typedef struct{
  float kp;            /* 比例：目标转速 rpm / 角度误差 deg */
  float ki;            /* 积分：rpm / (deg*s) */
  float kd;            /* 微分：rpm / (deg/s)，用速度反馈做阻尼 */
  float target_deg;    /* 目标角度 deg */
  float feedback_deg;  /* 当前角度 deg */
  float err_deg;       /* 走最短路径后的角度误差 deg */
  float integral;      /* 积分项，单位 rpm */
  float out_rpm;       /* 输出：速度环的目标转速 rpm */
  float out_max;       /* 目标转速限幅 rpm */
  float i_max;         /* 积分限幅 rpm */
  float deadband_deg;  /* 到位死区 deg */
  float rate_filt;     /* D 项用的低通角速度 deg/s */
  float d_alpha;       /* D 项低通系数 0~1，越小越平滑 */
}AnglePID_t;

/* 角度环实例与目标角度（定义在 pid_angle.c） */
extern AnglePID_t g_angle_pid;
extern volatile float g_target_deg;

/* 3508 编码器：一圈 8192 计数，0~8191 对应 0~360 度 */
float Encoder_To_Deg(uint16_t count);

void AnglePID_Init(AnglePID_t *p);
float AnglePID_Update(AnglePID_t *p, float feedback_deg, float rate_deg_s, float dt);

/* 角度模式入口：角度环 -> 速度环，结果写入 g_pid.out */
void PID_Angle_Control(float angle_deg, float rate_deg_s, float speed_rpm, float dt);

#endif
