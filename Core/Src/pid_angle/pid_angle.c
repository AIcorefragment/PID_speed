#include "pid_angle.h"
#include "../pid_speed/pid_speed.h"

/* 角度环实例（目标角度 g_target_deg 定义在主程序 main.c） */
AnglePID_t g_angle_pid;

/* 3508 编码器一圈 8192 计数，0~8191 对应 0~360 度 */
float Encoder_To_Deg(uint16_t count){
  return (float)count * (360.0f / 8192.0f);
}

void AnglePID_Init(AnglePID_t *p){
  p->kp = 3.0f;           /* 过冲/抖动就减小，归位慢就加大 */
  p->ki = 0.0f;           /* 消静差，过冲明显时先降 Ki */
  p->kd = 0.05f;          /* 阻尼项，先用很小；震动就再降或直接给 0 */
  p->target_deg = 0.0f;
  p->feedback_deg = 0.0f;
  p->err_deg = 0.0f;
  p->integral = 0.0f;
  p->out_rpm = 0.0f;
  p->out_max = 400.0f;    /* 输出给速度环的目标转速上限 rpm */
  p->i_max = 200.0f;
  p->deadband_deg = 0.5f; /* 到位死区，约 11 个编码器计数 */
  p->rate_filt = 0.0f;
  p->d_alpha = 0.2f;      /* 1 rpm 的反馈噪声经低通后才进 D 项 */
}

/* 位置式 PID，输出是速度环的目标转速 rpm。
   D 项直接用速度反馈（rpm*6 = deg/s），避免编码器差分的量化噪声。 */
float AnglePID_Update(AnglePID_t *p, float feedback_deg, float rate_deg_s, float dt){
  float p_term;
  float d_term;
  float out;

  p->feedback_deg = feedback_deg;
  p->err_deg = p->target_deg - feedback_deg;

  /* 误差归一到 ±180°，0°/360° 两侧走最短路径，拨过半圈也能就近归位 */
  while (p->err_deg > 180.0f){
    p->err_deg -= 360.0f;
  }
  while (p->err_deg < -180.0f){
    p->err_deg += 360.0f;
  }

  /* 到位死区：误差够小就断输出，避免在目标点上来回抖 */
  if ((p->err_deg <= p->deadband_deg) && (p->err_deg >= -p->deadband_deg)){
    p->integral = 0.0f;
    p->out_rpm = 0.0f;
    return 0.0f;
  }

  p_term = p->kp * p->err_deg;

  /* C620 的 speed_rpm 只有 1rpm 分辨率，直接乘 Kd 会把量化噪声
     放大成转速指令的抖动，所以先做一阶低通再进 D 项。 */
  p->rate_filt += p->d_alpha * (rate_deg_s - p->rate_filt);
  d_term = -p->kd * p->rate_filt;

  out = p_term + p->integral + d_term;

  /* 输出饱和、误差还在同方向推时不积分，防止 windup */
  if (((out < p->out_max) && (out > -p->out_max)) || ((p->err_deg * out) <= 0.0f)){
    p->integral += p->ki * p->err_deg * dt;
    if (p->integral > p->i_max){
      p->integral = p->i_max;
    }
    if (p->integral < -p->i_max){
      p->integral = -p->i_max;
    }
  }

  out = p_term + p->integral + d_term;

  if (out > p->out_max){
    out = p->out_max;
  } else if (out < -p->out_max){
    out = -p->out_max;
  }

  p->out_rpm = out;
  return out;
}

/* 角度模式：角度环算目标转速 -> 速度环算电流，结果在 g_pid.out */
void PID_Angle_Control(float angle_deg, float rate_deg_s, float speed_rpm, float dt){
  float speed_target;

  g_angle_pid.target_deg = g_target_deg;
  speed_target = AnglePID_Update(&g_angle_pid, angle_deg, rate_deg_s, dt);
  SpeedPID_Ramp(&g_pid, speed_target, dt);
  g_pid.out = SpeedPID_Update(&g_pid, speed_rpm, dt);
}
