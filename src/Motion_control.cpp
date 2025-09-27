#include "Motion_control.h"

/******************************     AS5600 角度传感器接口       *******************************/
AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};
// uint32_t AS5600_SCL[] = {PA6, PA4, PA2, PA0};
// uint32_t AS5600_SDA[] = {PA7, PA5, PA3, PA1};
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 10
float speed_as5600[4] = {0, 0, 0, 0};

/******************************     初始化 ADC       *******************************/
void MC_PULL_ONLINE_init()
{
    ADC_DMA_init();
}

/******************************     控制相关变量       *******************************/
float MC_PULL_stu_raw[4] = {0, 0, 0, 0};
int MC_PULL_stu[4] = {0, 0, 0, 0};
float MC_ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
// 0-离线 1-在线单微动触发 2-双微动触发 3-抖动
int MC_ONLINE_key_stu[4] = {0, 0, 0, 0};

// 电压控制相关常量
float PULL_voltage_up = 1.85f;   // 状态 压力高 红灯
float PULL_voltage_down = 1.45f; // 状态 压力低 蓝灯
// 微动触发控制相关常量
float MC_PULL_voltage_pull = 1.5f;
bool Assist_send_filament[4] = {false, false, false, false};
bool pull_state_old = false; // 上次触发状态——True：未触发，False：进料完成
bool is_backing_out = false;
uint64_t Assist_filament_time[4] = {0, 0, 0, 0};
uint64_t Assist_send_time = 1200; // 仅触发外侧后，送料时长
// 退料距离 单位 MM
float_t P1X_OUT_filament_meters = 200.0f;                  // 内置200mm 外置700mm
float_t last_total_distance[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 初始化退料开始时的距离
// bool filament_channel_inserted[4]={false,false,false,false};//通道是否插入
// 使用双微动
bool is_two = true;

/**
 * 通过 ADC 电压值来设定状态
 * 霍尔传感器 MC_PULL_stu_raw , 在线状态 MC_ONLINE_key_stu_raw
 */

#define BMCUMotor_version 1
#define use_flash_addr ((uint32_t)0x0800FA00)
struct alignas(4) Motor_save_struct
{
    uint32_t version = BMCUMotor_version;
    int pwm_zero[4] = {380, 380, 380, 380};
    uint64_t time_pull = 15000;

} motor_save;

void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 开启复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 999;  // 周期
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // 预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2完全映射-CH1-PA15/CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3部分映射-CH1-PB4/CH2-PB5
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4不映射-CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

#define PWM_lim 980

class MOTOR_PID
{
public:
    float P = 1;
    // float I = 1;
    float I = 10;
    // float D = 0;
    float D = 0.018;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;
    void init(float P_set, float I_set)
    {
        P = P_set;
        I = I_set;
        I_save = 0;
    }
    float caculate(float E, float time_E)
    {

        float I_save_set = (I_save + E * time_E);
        if ((abs(I * I_save_set) < pid_range / 2)) // 对I限幅
            I_save = I_save_set;                   // 线性I系数

        float ouput_buf = P * (E + I * (I_save) + D * (E - E_last) / time_E);
        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};
class _MOTOR_CONTROL
{
public:
    int motion = 0;
    int CHx = 0;
    int pwm_zero = 380;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID;

    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = 0;
    }
    void set_motion(int _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        motion = _motion;
    }
    void set_motion_add(int _motion, uint64_t over_time)
    {
        motor_stop_time += over_time;
        motion = _motion;
    }
    int get_motion()
    {
        return motion;
    }
    void set_pwm_zero(int _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void run(float now_speed)
    {
        uint64_t time_now = get_time64();
        static uint64_t time_set_speed = 0;
        static uint64_t time_last = 0;
        float speed_set = 0;
        if (time_now >= motor_stop_time)
        {
            motion = 0;
        }
        if ((get_filament_online(CHx) == false))
        {
            set_filament_motion(CHx, idle);
            Assist_send_filament[CHx] = true; // 某通道离线后才可触发辅助进料一次
            countdownStart[CHx] = 0;          // 清空倒计时            
        }
        if (get_filament_motion(CHx) == idle) // 在空闲状态
        {

            if (Assist_send_filament[CHx] && is_two)
            { // 允许状态，尝试辅助进料
                if (MC_ONLINE_key_stu[CHx] == 2)
                {                   // 触发外侧微动
                    speed_set = 40; // 驱动送料
                }
                if (MC_ONLINE_key_stu[CHx] == 1)
                { // 同时触发双微动，准备停机
                    if (countdownStart[CHx] == 0)
                    { // 启动倒计时
                        countdownStart[CHx] = get_time64();
                    }
                    uint64_t now = get_time64();
                    if (now - countdownStart[CHx] >= Assist_send_time) // 倒计时
                    {
                        speed_set = 0;                     // 停止电机
                        Assist_send_filament[CHx] = false; // 达成条件，完成一轮辅助进料。
                    }
                    else
                    {
                        // 驱动送料
                        speed_set = 40;
                    }
                }
            }
            else
            {
                // 已经触发过，或微动触发在其他状态
                if (MC_ONLINE_key_stu[CHx] != 0 && MC_PULL_stu[CHx] == 2)
                { // 如果滑块被人为拉动，做出对应响应
                    // x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - 1.65, time_E);
                    speed_set = -40;
                }
                else if (motion == 0)
                { // 否则，保持停机
                    speed_set = 0;
                    PID.clear();
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
            }
        }
        else
        {
            if (motion == 99) // 刹车
            {
                speed_set = 0;
            }
            if (motion == 1) // send 370 40  130 15
            {
                speed_set = 40;
            }
            else if (motion == 2) // over pressure
            {
                speed_set = 5;
            }
            else if (motion == -3) //  pull 进料重试
            {
                speed_set = -40;
            }
            else if (motion == -4) // slowly pull
            {
                speed_set = -15;
            }
            else if (motion == -1 || motion == -2) // pull 370 70 130 18
            {
                speed_set = -60;
            }
            else if (motion == 100) // onuse send 370 15 130 10
            {
                speed_set = 15;
            }
            else if (motion == 200) // onuse send 370 25 130 15
            {
                speed_set = 25;
            }
            else if (motion == -100) // onuse pull 370 15 130 10
            {
                speed_set = -50;
            }
            else if (motion == -200) // onuse pull 370 25 130 15
            {
                speed_set = -70;
            }
        }

        float x = PID.caculate(now_speed - speed_set, (float)(time_now - time_last) / 1000);
        if (x > 1)
            x += pwm_zero;
        else if (x < 1)
            x -= pwm_zero;
        else
            x = 0;
        if (x > PWM_lim)
            x = PWM_lim;
        if (x < -PWM_lim)
            x = -PWM_lim;
        if (now_speed > 0.5 || motion == 0 || now_speed < -0.5)
        {
            time_set_speed = time_now + 5000;
        }
        if (time_set_speed < time_now && time_set_speed != 0)
        {
            if (x > 800)
                x = 0;
            else if (x < -800)
                x = 0; // 防止电机卡死过热
        }
        Motion_control_set_PWM(CHx, -x);
        time_last = time_now;
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};

// uint8_t ONLINE_key_stu[4] = {0, 0, 0, 0};
// uint64_t ONLINE_key_stu_count[4] = {0, 0, 0, 0};
// uint8_t ONLINE_key_change[4] = {0, 0, 0, 0};
//  uint64_t ONLINE_key_change_count[4] = {0, 0, 0, 0};
void MC_PULL_ONLINE_read()
{
    float *data = ADC_DMA_get_value();
    MC_PULL_stu_raw[3] = data[0];
    MC_ONLINE_key_stu_raw[3] = data[1];
    MC_PULL_stu_raw[2] = data[2];
    MC_ONLINE_key_stu_raw[2] = data[3];
    MC_PULL_stu_raw[1] = data[4];
    MC_ONLINE_key_stu_raw[1] = data[5];
    MC_PULL_stu_raw[0] = data[6];
    MC_ONLINE_key_stu_raw[0] = data[7];

    for (int i = 0; i < 4; i++)
    {
        /*
        if (i == 0){
            DEBUG_MY("MC_PULL_stu_raw = ");
            DEBUG_float(MC_PULL_stu_raw[i],3);
            DEBUG_MY("  MC_ONLINE_key_stu_raw = ");
            DEBUG_float(MC_ONLINE_key_stu_raw[i],3);
            DEBUG_MY("  通道：");
            DEBUG_float(i,1);
            DEBUG_MY("   \n");
        }
        */
        if (MC_PULL_stu_raw[i] > 2) // 大于2V,表示压力过高
        {
            MC_PULL_stu[i] = 2;
        }
        else if (MC_PULL_stu_raw[i] < 1.3) // 小于1.3V，表示压力过低
        {
            MC_PULL_stu[i] = -2;
        }
        else if (MC_PULL_stu_raw[i] > PULL_voltage_up) // 大于1.85V,表示压力高
        {
            MC_PULL_stu[i] = 1;
        }
        else if (MC_PULL_stu_raw[i] < PULL_voltage_down) // 小于1.45V，表示压力低
        {
            MC_PULL_stu[i] = -1;
        }
        else // 1.4~1.7之间，在正常误差范围内，无需动作
        {
            MC_PULL_stu[i] = 0;
        }

        /*在线状态*/

        // 耗材在线判断 , is_two 考虑弃用
        if (is_two == false)
        {
            // 大于1.65V，为耗材在线，高电平.
            if (MC_ONLINE_key_stu_raw[i] > 1.55)
            {
                MC_ONLINE_key_stu[i] = 1;
            }
            else
            {
                MC_ONLINE_key_stu[i] = 0;
            }
        }
        else
        {
            // DEBUG_MY(MC_ONLINE_key_stu_raw);
            // 双微动
            if (MC_ONLINE_key_stu_raw[i] < 0.6f)
            { // 小于则离线.
                MC_ONLINE_key_stu[i] = 0;
            }
            else if ((MC_ONLINE_key_stu_raw[i] < 1.8f) & (MC_ONLINE_key_stu_raw[i] > 1.4f))
            { // 仅触发1个微动，需辅助进料
                MC_ONLINE_key_stu[i] = 1;
            }
            else if (MC_ONLINE_key_stu_raw[i] > 1.8f)
            { // 双微动同时触发, 在线状态
                MC_ONLINE_key_stu[i] = 2;
            }
            else if (MC_ONLINE_key_stu_raw[i] < 1.4f)
            { // 仅触发内侧微动 , 需确认是缺料还是抖动.
                MC_ONLINE_key_stu[i] = 3;
            }
        }
    }
}

void Motion_control_set_PWM(uint8_t CHx, int PWM)
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}

void MOTOR_get_pwm_zero()
{
    int pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 200; pwm < 500; pwm += 20)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 20)
                {
                    pwm_zero[index] = pwm - 80;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, pwm);
                }
                last_angle[index] = MC_AS5600.raw_angle[index];
                if (pwm == 500)
                {
                    pwm_zero[index] = pwm - 20;
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
            delay(20);
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}

void MOTOR_set_pwm_zero(int pwm)
{
    motor_save.pwm_zero[0] = pwm;
    motor_save.pwm_zero[1] = pwm;
    motor_save.pwm_zero[2] = pwm;
    motor_save.pwm_zero[3] = pwm;
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(motor_save.pwm_zero[index]);
    }
}

bool Motor_read()
{
    Motor_save_struct *ptr = (Motor_save_struct *)(use_flash_addr);
    if (ptr->version == BMCUMotor_version)
    {
        memcpy(&motor_save, ptr, sizeof(motor_save));
        return true;
    }
    return false;
}
bool motor_need_to_save = false;
void Motor_set_need_to_save()
{
    motor_need_to_save = true;
}
void Motor_save()
{
    Flash_saves(&motor_save, sizeof(motor_save), use_flash_addr);
    motor_need_to_save = false;
}
bool Motor_need_to_save()
{
    return motor_need_to_save;
}

void Motor_init()
{
    bool _init_ready = Motor_read();
    if (!_init_ready)
    {
        motor_save.pwm_zero[0] = 380;
        motor_save.pwm_zero[1] = 380;
        motor_save.pwm_zero[2] = 380;
        motor_save.pwm_zero[3] = 380;
        motor_save.time_pull = 12000;
        Motor_save();
    }

    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(motor_save.pwm_zero[index]);
    }
}

void Motion_control_init()
{
    MC_PWM_init();
    // MC_PULL_key_init();
    // MC_ONLINE_key_init();
    MC_PULL_ONLINE_init();
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    Motor_init();
}

void AS5600_distance_updata()
{
    static int32_t distance_save[4] = {0, 0, 0, 0};
    static uint64_t time_last = 0;
    uint64_t time_now = get_time64();
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1))
        {
            distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm
        distance_save[i] = now_distance;
        float T = (float)(time_now - time_last);
        float speedx = distance_E / T * 1000;
        T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx * (1 - T) + speed_as5600[i] * T; // mm/s
        if (get_filament_motion(i) != on_use || distance_E > 0)
            add_filament_meters(i, distance_E / 1000);
    }
    time_last = time_now;
}
uint64_t send_count[4] = {0, 0, 0, 0};
uint64_t sendcheck_count[4] = {0, 0, 0, 0}; // 长回抽
uint64_t senddelay_count[4] = {0, 0, 0, 0}; // 长回抽
uint64_t pulldelay_count[4] = {10, 10, 10, 10};
uint64_t pullcheck_count[4] = {0, 0, 0, 0};
uint64_t pullcheck[4] = {0, 0, 0, 0}; // 当前bmcu通道使用标记
void Sendcount_clear(uint8_t CHx)
{
    sendcheck_count[CHx] = 0;
    senddelay_count[CHx] = 0;
}
void Pullcount_clear(uint8_t CHx)
{
    pullcheck_count[CHx] = 0;
    pulldelay_count[CHx] = 0;
}
void Pullcheck_set(uint8_t CHx, int n)
{
    pullcheck[CHx] = n;
}
void Pullcheck_clear()
{
    for (int i = 0; i < 4; i++)
    {
        pullcheck[i] = 0;
    }
}
bool Pullcheck(uint8_t CHx)
{
    for (int i = 0; i < 4; i++)
    {
        if (CHx != i)
        {
            if (pullcheck[i] != 0)
                return true;
        }
    }
    return false;
}
bool Bmcucheck()
{
    for (int i = 0; i < 4; i++)
    {
        if (MC_PULL_stu[i] < 0 || MC_ONLINE_key_stu[i] != 1)
            return true;
    }
    return false;
}
uint8_t lastnum = 0;

void MOTOR_set_time_pull(uint64_t time1)
{
    motor_save.time_pull = time1;
}

void motor_motion_run()
{
    bool select = Bmcu_select();
    uint8_t num = get_now_filament_num();
    uint64_t time_now = get_time64();
    uint64_t time_pull = 500;
    if (motor_save.time_pull > 14000)
        time_pull = motor_save.time_pull / 2;
    uint64_t time_set = motor_save.time_pull - time_pull;
    uint64_t time_set_2 = time_now + time_set;
    uint64_t time_set_3 = time_now + 5000;

    if (!Pullcheck(num))
    {
        if (senddelay_count[num] == 0)
            senddelay_count[num] = 1;
        if (pulldelay_count[num] == 0)
            pulldelay_count[num] = 1;
    }
    if (num != lastnum)
    {
        if (pullcheck[lastnum] == 2)
        {
            if (MOTOR_CONTROL[lastnum].get_motion() == -2)
                MOTOR_CONTROL[lastnum].set_motion_add(-1, time_set); // 短回抽通道 退料
            else if (MOTOR_CONTROL[lastnum].get_motion() == 0)
                MOTOR_CONTROL[lastnum].set_motion(-1, time_set);
        }
        Sendcount_clear(lastnum);
        Pullcount_clear(lastnum);
        lastnum = num;
        Pullcheck_clear();
    }

    if (Bmcu_reset())
    {
        for (int i = 0; i < 4; i++)
        {
            if (pullcheck[i] == 2)
            {
                if (get_filament_motion(i) == idle)
                    MOTOR_CONTROL[i].set_motion(-1, time_set); // 所有通道退回五通后
            }
        }
        Pullcheck_clear();
        Bmcu_set_no_reset();
    }
    if (get_filament_online(num))
    {
        switch (get_filament_motion(num))
        {
        case need_send_out:
            RGB_set(num, 0x00, 0xFF, 0x00);
            if (senddelay_count[num] == 0)
                senddelay_count[num] = time_set_2;
            else if (senddelay_count[num] == 1)
                senddelay_count[num] = time_set_3;
            if (senddelay_count[num] < time_now && MOTOR_CONTROL[num].get_motion() != -3)
            {
                // if (sendcheck_count[num] == 0)

                // if (sendcheck_count[num] > time_now && ONLINE_key_change[num] == 0)
                if (MC_PULL_stu[num] < 1)
                {
                    MOTOR_CONTROL[num].set_motion(1, 100);
                    sendcheck_count[num] = 0;
                }
                else
                {
                    MOTOR_CONTROL[num].set_motion(99, 100);
                }
            }
            if (send_count[num] > time_now && send_count[num] < time_now + 1000)
            {
                if (MC_PULL_stu[num] >= 1)
                    MOTOR_CONTROL[num].set_motion(-3, 1500);
            }
            else if (MC_PULL_stu[num] >= 1 && sendcheck_count[num] == 0) // 进料重试
            {
                send_count[num] = time_now + 2500;
                sendcheck_count[num] = 1;
            }

            break;
        case need_pull_back:
            RGB_set(num, 0xFF, 0x00, 0xFF);
            if (pulldelay_count[num] == 0)
                pulldelay_count[num] = time_set_3 + 3000;
            if (pulldelay_count[num] > time_now)
            {
                MOTOR_CONTROL[num].set_motion(-1, motor_save.time_pull); // 退料时间调整
            }
            else
            {
                if (pullcheck_count[num] == 0)
                    pullcheck_count[num] = time_set_3 + 3000;
                if (pullcheck_count[num] > time_now)
                {
                    MOTOR_CONTROL[num].set_motion(-2, time_pull);
                }
                else
                {
                    MOTOR_CONTROL[num].set_motion(0, 100);
                    if (pullcheck_count[num] < time_now - 5000)
                    {
                        pullcheck_count[num] = 0;
                        set_filament_motion(num, idle); // 防止卡回抽状态
                    }
                }
            }
            break;
        case on_use:
            Pullcount_clear(num); // 注销 短回抽
            if (MOTOR_CONTROL[num].get_motion() == 1)
            {
                MOTOR_CONTROL[num].set_motion(99, 150); // 停电机 清空pid
            }
            else if (MOTOR_CONTROL[num].get_motion() == 99)
            {
                MOTOR_CONTROL[num].set_motion(2, 1000); // 保持压力延迟1000ms
            }
            else if (MOTOR_CONTROL[num].get_motion() != 2 || MC_PULL_stu[num] < 0)
            {
                if (MC_PULL_stu[num] == -1)
                    MOTOR_CONTROL[num].set_motion(100, 100);
                else if (MC_PULL_stu[num] == -2)
                    MOTOR_CONTROL[num].set_motion(200, 100);
                else if (MC_PULL_stu[num] == 1)
                    MOTOR_CONTROL[num].set_motion(-100, 100);
                else if (MC_PULL_stu[num] == 2)
                    MOTOR_CONTROL[num].set_motion(-100, 100);
            }

            RGB_set(num, 0xFF, 0xFF, 0xFF);
            break;
        case pre_pull:
            if (MC_PULL_stu[num] < 0)
                MOTOR_CONTROL[num].set_motion(0, 100);
            else if (MC_PULL_stu[num] > 0)
                MOTOR_CONTROL[num].set_motion(-100, 100);
            else
                MOTOR_CONTROL[num].set_motion(-4, 100);
            break;
        case idle:
            Sendcount_clear(num);
            if (MOTOR_CONTROL[num].get_motion() == -1)
                Pullcheck_set(num, 1);
            else if (MOTOR_CONTROL[num].get_motion() == -2)
                Pullcheck_set(num, 2);
            else if (MOTOR_CONTROL[num].get_motion() == 99)
                MOTOR_CONTROL[num].set_motion(-3, 1000);
            RGB_set(num, 0x00, 0x00, 0x37);
            break;
        }
    }
    if (!select && MOTOR_CONTROL[num].get_motion() < 0)
    {
        Pullcheck_set(num, 2);
        MOTOR_CONTROL[num].set_motion(0, 100);
    }
    for (int i = 0; i < 4; i++)
    {
        if (i != num)
            MOTOR_CONTROL[i].run(speed_as5600[i]);
    }
    MOTOR_CONTROL[num].run(speed_as5600[num]);
}
bool bmcuset_en = false;
uint64_t time_bmcuset = 0;
uint64_t time_led = 0;
bool Bmcu_set()
{
    return bmcuset_en;
}
void Motion_control_run(int error)
{
    // MC_PULL_key_read();
    // MC_ONLINE_key_read();
    MC_PULL_ONLINE_read();

    AS5600_distance_updata();
    uint64_t time_now = get_time64();

    for (int i = 0; i < 4; i++)
    {

        if ((MC_ONLINE_key_stu[i] != 0) && (MC_AS5600.online[i] == true))
        {
            set_filament_online(i, true);
        }
        else
        {
            set_filament_online(i, false);
        }
    }
    if (!Bmcucheck() && time_bmcuset == 0)
    {
        time_bmcuset = time_now + 10000;
    }

    if (time_bmcuset > time_now && time_bmcuset < time_now + 8000)
        bmcuset_en = true;
    else
        bmcuset_en = false;

    if (bmcuset_en) // 手动设置bmcu编号
    {
        uint8_t num = 0;
        for (int i = 1; i < 5; i++)
        {
            if (MC_PULL_stu[i - 1] == 2 && MC_ONLINE_key_stu[i - 1] == 0) // 拔出缓冲且拔出耗材
                num += i;
        }

        if (time_bmcuset > time_now && time_bmcuset < time_now + 2000 && num != 0)
        {
            Bmcu_set_num(num - 1);
            time_bmcuset = 0;
        }
    }

    if (error)
    {
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            if (MC_PULL_stu[i] == -2)
            {
                RGB_set(i, 0xFF, 0x00, 0x00);
                if (MC_ONLINE_key_stu[i] != 0)
                {
                    RGB_set(i, 0xFF, 0x00, 0xFF);
                }
            }
            else if (MC_ONLINE_key_stu[i] != 0)
            {
                RGB_set(i, 0x00, 0x00, 0xFF);
            }
            else
            {
                RGB_set(i, 0x00, 0x00, 0x00);
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            if (get_filament_online(i))
                RGB_set(i, 0x00, 0x00, 0x37);
            else
                RGB_set(i, 0x37, 0x00, 0x00);
        }
    }

    motor_motion_run();
}
