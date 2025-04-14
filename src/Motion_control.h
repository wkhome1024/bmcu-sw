#pragma once

#include "main.h"

extern bool Bmcu_set();
extern void Motion_control_init();
extern void Motion_control_set_PWM(uint8_t CHx,int PWM);
extern void Motion_control_run(int error);
extern void Sendcount_clear(uint8_t CHx);