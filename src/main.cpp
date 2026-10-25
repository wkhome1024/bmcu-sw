#include <Arduino.h>
#include "main.h"

#include "WS2812.h"
WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, PD1);
    RGBOUT[3].init(2, PB0);
    RGBOUT[2].init(2, PB1);
    RGBOUT[1].init(2, PA8);
    RGBOUT[0].init(2, PA11);
}
void RGB_update()
{
    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}
void RGB_set(unsigned char CHx, unsigned char R, unsigned char G, unsigned char B)
{
    RGBOUT[CHx].set_RGB(R, G, B, 1);
}
void RGB_pull_check(unsigned char CHx, unsigned char R, unsigned char G, unsigned char B)
{
    RGBOUT[CHx].set_RGB(R, G, B, 0);
}
extern void BambuBUS_UART_Init();
extern void send_uart(const unsigned char *data, uint16_t length);

void setup()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    BambuBus_init();
    Motion_control_init();
    RGB_init();
    SYS_RGB.set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[0].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[1].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[2].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[3].set_RGB(0x00, 0x00, 0x00, 0);
    RGB_update();
    delay(1);
}

// extern double distance_count;
uint8_t R = 0, G = 0, B = 0;
uint8_t T_to_tangle(uint32_t time)
{
    time = time % 256;
    if (time > 128)
        return 512 - time * 2;
    else
        return time * 2;
}

int error = 0;
uint64_t error_time = 0;
constexpr uint64_t kControlPeriodMs = 25; // 控制环固定 100 Hz，与总线帧率解耦
constexpr uint64_t kLedPeriodMs = 500;
void loop()
{
    uint64_t next_control = 0;
    uint64_t next_led = 0;

    while (1)
    {
        package_type stu = BambuBus_run();
        uint64_t time_now = get_time64();
        if (stu != BambuBus_package_NONE) // have data/offline
        {
            if (stu == BambuBus_package_ERROR) // offline
            {
                if (error_time < time_now - 1000)
                {
                    error_time = time_now + 1000;
                }
                else if (time_now > error_time && error_time > time_now - 1000)
                {
                    error_time = 0;
                    //error = -1;                    
                }
                SYS_RGB.set_RGB(0x30, 0x00, 0x00, 0);
                // RGB_update();
            }
            else // have data
            {
                error = 0;                
                if (stu == BambuBus_package_heartbeat)
                {
                    if (Bmcu_set())
                        SYS_RGB.set_RGB(0xf9, 0xa8, 0x46, 0);
                    else
                        SYS_RGB.set_RGB(0x00, 0x00, 0x30, 0);
                    // RGB_update();
                }
            }
        }

        // 控制节拍只由本地时基决定：即使总线静默，电机闭环仍按 100 Hz 运行。
        if (time_now >= next_control)
        {
            Motion_control_run(error);
            next_control = time_now + kControlPeriodMs;
        }
        if (time_now >= next_led)
        {
            RGB_update();
            next_led = time_now + kLedPeriodMs;
        }
        delay(1);
    }
}
