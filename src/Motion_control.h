#pragma once

#include "main.h"

extern bool Bmcu_set();
extern void Motion_control_init();
extern void Motion_control_set_PWM(uint8_t CHx,int PWM);
extern void Motion_control_run(int error);
extern void Sendcount_clear(uint8_t CHx);
extern void MOTOR_set_time_pull(bool select ,uint64_t time1);
extern void MOTOR_set_pwm_zero(int pwm);
extern void Motor_set_need_to_save();
extern void Motor_save();
extern bool Motor_need_to_save();
extern void MOTOR_get_pwm_zero();