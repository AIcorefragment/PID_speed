#include "pid_speed.h"

/* 速度环实例（目标转速 g_target_rpm 定义在主程序 main.c） */
SpeedPID_t g_pid;

void SpeedPID_Init(SpeedPID_t *p){
  p->kp = 6.0f;         /* 先给保守值，振荡就减小，响应慢就加大 */
  p->ki = 15.0f;
  p->kd = 0.0f;         /* 速度环一般不用 D，噪声大 */
  p->target = 0.0f;
  p->target_ramp = 0.0f;
  p->feedback = 0.0f;
  p->fb_prev = 0.0f;
  p->err = 0.0f;
  p->err_prev = 0.0f;
  p->integral = 0.0f;
  p->out = 0.0f;
  p->out_max = 16384.0f;   /* 20A */
  p->i_max = 12000.0f;     /* 积分最多占约 73% 输出 */
  p->acc = 6000.0f;        /* 目标斜坡 6000 rpm/s，归位更快 */
}

/* 目标转速斜坡，避免阶跃给定导致电流冲击 */
void SpeedPID_Ramp(SpeedPID_t *p, float target, float dt){
  float diff;
  float step;

  p->target = target;
  diff = target - p->target_ramp;
  step = p->acc * dt;

  if (diff > step){
    p->target_ramp += step;
  } else if (diff < -step){
    p->target_ramp -= step;
  } else {
    p->target_ramp = target;
  }
}

/* 位置式 PI + 条件积分抗饱和，输出单位是 C620 的电流计数 */
float SpeedPID_Update(SpeedPID_t *p, float feedback, float dt){
  float p_term;
  float d_term;
  float out;

  p->feedback = feedback;
  p->err = p->target_ramp - feedback;

  p_term = p->kp * p->err;
  d_term = -p->kd * (feedback - p->fb_prev) / dt;
  p->fb_prev = feedback;

  out = p_term + p->integral + d_term;

  /* 输出已经饱和、误差还在同方向时不积分，防止 windup */
  if (((out < p->out_max) && (out > -p->out_max)) || ((p->err * out) <= 0.0f)){
    p->integral += p->ki * p->err * dt;
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
  }
  if (out < -p->out_max){
    out = -p->out_max;
  }

  p->out = out;
  return out;
}

/* 单独速度模式：g_target_rpm -> 速度环 -> g_pid.out */
void PID_Speed_Control(float speed_rpm, float dt){
  SpeedPID_Ramp(&g_pid, g_target_rpm, dt);
  g_pid.out = SpeedPID_Update(&g_pid, speed_rpm, dt);
}
