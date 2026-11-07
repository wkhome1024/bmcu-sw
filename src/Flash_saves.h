#pragma once

#include <stdint.h>

// 从机只保存“设置参数”，不保存材料属性/运行状态/里程。
struct Settings
{
    uint8_t bmcu = 0;           // 从机编号
    uint8_t reserved = 0;
    uint16_t pwm_zero = 320;    // 四通道统一 PWM 零点
    uint16_t pull_ms = 4000;    // 二段退料时间
    uint16_t pull_t1_ms = 4000; // 一段退料时间
};
static_assert(sizeof(Settings) == 8, "Settings layout changed");

// 启动时扫描 0x0800F000；无有效记录则擦页写入默认值。
bool Settings_load();

// 内存中的当前设置。
Settings &Settings_get();

// 追加一条记录；成功后清 dirty。写满 128 条后擦页从 Slot 0 重建。
bool Settings_save();

void Settings_mark_dirty();
bool Settings_is_dirty();
