#include "BambuBus.h"
#include "CRC16.h"
#include "CRC8.h"
CRC16 crc_16;
CRC8 crc_8;

uint8_t BambuBus_data_buf[500];
int BambuBus_have_data = 0;
uint16_t BambuBus_address = 0x0700;
// uint8_t AMS_num = 1;
bool bmcu_reset = false;
bool Bmcu_filament_flag = false;
bool Bmcu_select_flag = false;
uint64_t Bmcu_selest_count = 0;
struct _filament
{
    float meters = 0;
    _filament_status statu = online;
    // printer_set
    _filament_motion_state_set motion_set = idle;
    uint16_t pressure = 0;
};

// 运行时状态：不进入 Flash。
struct alignas(4) runtime_save_struct
{
    _filament filament[4];
    int BambuBus_now_filament_num = 0;
    uint8_t bmcu = 0;
} data_save;

bool Bambubus_read()
{
    Settings_load();
    data_save.bmcu = Settings_get().bmcu;
    for (int i = 0; i < 4; i++)
    {
        data_save.filament[i].meters = 0;
        data_save.filament[i].pressure = 0xFFFF;
    }
    return true;
}

int get_now_filament_num()
{
    return data_save.BambuBus_now_filament_num;
}

void reset_filament_meters(int num)
{
    if (num < 0 || num >= 4)
        return;
    data_save.filament[num].meters = 0;
}
void add_filament_meters(int num, float meters)
{
    if (num < 0 || num >= 4 || !(meters == meters))
        return;
    data_save.filament[num].meters += meters;
}
float get_filament_meters(int num)
{
    return data_save.filament[num].meters;
}
void set_filament_online(int num, bool if_online)
{
    if (if_online)
        data_save.filament[num].statu = online;
    else
        data_save.filament[num].statu = offline;
}

bool get_filament_online(int num)
{
    if (data_save.filament[num].statu == offline)
    {
        return false;
    }
    else
    {
        return true;
    }
}
void set_filament_motion(int num, _filament_motion_state_set motion)
{
    data_save.filament[num].motion_set = motion;
    data_save.BambuBus_now_filament_num = num;
}

void set_now_filament_num(int num)
{
    data_save.BambuBus_now_filament_num = num;
}

_filament_motion_state_set get_filament_motion(int num)
{
    return data_save.filament[num].motion_set;
}

bool filament_check()
{
    for (int i = 0; i < 4; i++)
    {
        if (data_save.filament[i].motion_set != idle)
        {
            return true;
        }
    }
    return false;
}

uint8_t buf_X[500];
CRC8 _RX_IRQ_crcx(0x39, 0x66, 0x00, false, false);
void inline RX_IRQ(unsigned char _RX_IRQ_data)
{
    static int _index = 0;
    static uint8_t data_length_index = 0;
    static uint8_t data_CRC8_index;
    unsigned char data = _RX_IRQ_data;

    if (_index == 0)
    {
        if (data == 0x9D)
        {
            BambuBus_data_buf[0] = 0x9D;
            _RX_IRQ_crcx.restart();
            _RX_IRQ_crcx.add(0x9D);
            _index = 1;
        }
        return;
    }
    else
    {
        BambuBus_data_buf[_index] = data;
        if (_index == 1)
        {
            data_length_index = data;
            data_CRC8_index = data_length_index - 1;
        }
        if (_index < data_CRC8_index)
        {
            _RX_IRQ_crcx.add(data);
        }
        else if (_index == data_CRC8_index)
        {
            if (data != _RX_IRQ_crcx.calc())
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= data_length_index)
        {
            _index = 0;
            memcpy(buf_X, BambuBus_data_buf, data_length_index);
            BambuBus_have_data = data_length_index;
        }
        if (_index >= 50)
        {
            _index = 0;
        }
    }
}

#include <stdio.h>

DMA_InitTypeDef Bambubus_DMA_InitStructure;
void send_uart(const unsigned char *data, uint16_t length)
{
    // Configure DMA1 channel 4 for USART1 TX
    DMA1_Channel4->CFGR &= (uint16_t)(~DMA_CFGR1_EN); // 关闭通道
    DMA1_Channel4->MADDR = (uint32_t)data;            // 更新内存地址
    DMA1_Channel4->CNTR = length;                     // 更新传输长度
    DMA1_Channel4->CFGR |= DMA_CFGR1_EN;              // 重新使能
    GPIOA->BSHR = GPIO_Pin_12;
    // Enable USART1 DMA send
    USART1->CTLR3 |= USART_DMAReq_Tx;
}

void BambuBUS_UART_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART1 TX-->A.9   RX-->A.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // DE
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BCR = GPIO_Pin_12;

    USART_InitStructure.USART_BaudRate = 512000;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure DMA1 channel 4 for USART1 TX
    Bambubus_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    Bambubus_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    Bambubus_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    Bambubus_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    Bambubus_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    Bambubus_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    Bambubus_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    Bambubus_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    Bambubus_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    Bambubus_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    Bambubus_DMA_InitStructure.DMA_BufferSize = 0;
    DMA_Init(DMA1_Channel4, &Bambubus_DMA_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        RX_IRQ(USART_ReceiveData(USART1));
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        GPIOA->BCR = GPIO_Pin_12;
    }
}
void BambuBus_init()
{
    Bambubus_read(); // 只加载 Settings，不保留材料属性/旧配置
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);

    for (auto &i : data_save.filament)
    {
#ifdef _Bambubus_DEBUG_mode_
        i.statu = online;
#else
        i.statu = offline;
#endif // DEBUG

        i.motion_set = idle;
    }

    BambuBUS_UART_Init();
}

bool package_check_crc16(uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}
bool need_debug = false;
void package_send_with_crc(uint8_t *data, int data_length)
{
    data[0] = 0x7D;
    data[1] = data_length;
    crc_8.restart();
    for (auto i = 0; i < data_length - 1; i++)
    {
        crc_8.add(data[i]);
    }
    uint8_t num = crc_8.calc();
    data[(data_length - 1)] = num;
    send_uart(data, data_length);
}

uint8_t packge_send_buf[500];

#pragma pack(push, 1) // 将结构体按1字节对齐
struct long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas;
    uint16_t data_length;
};
#pragma pack(pop) // 恢复默认对齐

void Bambubus_long_package_send(long_packge_data *data)
{
    packge_send_buf[0] = 0x3D;
    packge_send_buf[1] = 0x00;
    data->package_length = data->data_length + 15;
    memcpy(packge_send_buf + 2, data, 11);
    memcpy(packge_send_buf + 13, data->datas, data->data_length);
    package_send_with_crc(packge_send_buf, data->data_length + 15);
}

void Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // +2byte CRC16
}

long_packge_data printer_data_long;
package_type get_packge_type(unsigned char *buf, int length)
{
    /*if (package_check_crc16(buf, length) == false)
    {
        return BambuBus_package_NONE;
    }*/
    if (buf[0] == 0x9D)
    {

        switch (buf[2])
        {
        case 0x03:
            return BambuBus_package_filament_motion_short;
        case 0x04:
            return BambuBus_package_filament_motion_long;
        case 0x05:
            return BambuBus_package_online_detect;
        case 0x06:
            return BambuBus_package_REQx6;
        case 0x07:
            return BambuBus_package_NFC_detect;
        case 0x08:
            return BambuBus_package_set_filament;
        case 0x20:
            return BambuBus_package_heartbeat;
        default:
            return BambuBus_package_ETC;
        }
    }
    return BambuBus_package_NONE;
}

uint8_t get_filament_left_char(uint8_t AMS_num)
{
    uint8_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        if (data_save.filament[i].statu == online)
        {
            data |= (0x1 << i) << i; // 1<<(2*i)

            if (Filament_channel_onpull(i))
            {
                data |= (0x2 << i) << i; // 2<<(2*i)
            }
        }
    }
    return data;
}

void set_motion_res_datas(unsigned char *set_buf, unsigned char AMS_num, unsigned char read_num, unsigned char statu_flags)
{
    // unsigned char statu_flags = buf[6];

    // unsigned char fliment_motion_flag = buf[8];
    float meters = 0;
    uint8_t flagx = 0x00;
    if (read_num != 0xFF)
    {
        if (data_save.filament[read_num].meters >= 0 && data_save.filament[read_num].meters < 400)
            meters = data_save.filament[read_num].meters;
        else
            data_save.filament[read_num].meters = 0;
    }
    else if (read_num == 0xFF && statu_flags == 0x03)
    {
        meters = data_save.filament[data_save.BambuBus_now_filament_num].meters;
        read_num = data_save.BambuBus_now_filament_num;
    }
    // meters = data_save.filament[read_num].meters;
    memcpy(set_buf + 6, &meters, sizeof(meters));
    for (int i = 0; i < 4; i++)
    {
        // pressure = data_save.filament[read_num].pressure;
        if ((data_save.filament[i].motion_set == idle)) // idle
        {
            flagx = 0x00;
        }
        else if ((data_save.filament[i].motion_set == need_pull_back)) // pull back
        {
            flagx = 0x01;
        }
        else if ((data_save.filament[i].motion_set == need_send_out)) // sending
        {
            flagx = 0x02;
        }
        else if ((data_save.filament[i].motion_set == pre_pull)) // pre_pull
        {
            flagx = 0x03;
        }
        else if ((data_save.filament[i].motion_set == on_use)) // on use
        {
            flagx = 0x04;
        }
        set_buf[i + 2] = flagx;
        set_buf[i + 2] |= (GET_pull_state(i) & 0xF0); // pull state
    }
    set_buf[0] = AMS_num;
    set_buf[1] = read_num;
    set_buf[10] = get_filament_left_char(AMS_num);
}

bool set_motion(unsigned char AMS_num, unsigned char read_num, unsigned char statu_flags, unsigned char fliment_motion_flag)
{
    static uint64_t time_last = 0;
    static uint64_t idle_count = 0;
    static uint64_t pre_pull_count = 0;
    uint64_t time_now = get_time64();
    uint64_t time_used = time_now - time_last;
    time_last = time_now;
    if (BambuBus_address == 0x700) // AMS08
    {
        if (read_num < 4)
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00
            {
                uint8_t numx = read_num;
                if (data_save.BambuBus_now_filament_num != numx) // on change
                {
                    for (auto i = 0; i < 4; i++)
                    {
                        data_save.filament[i].motion_set = idle;
                        data_save.filament[i].pressure = 0xFFFF;
                    }
                    data_save.BambuBus_now_filament_num = numx;
                }
                data_save.filament[read_num].motion_set = need_send_out;
                data_save.filament[read_num].pressure = 0x4700;
                pre_pull_count = 0;
            }
            else if ((statu_flags == 0x09)) // 09 A5 / 09 3F
            {
                if (data_save.filament[read_num].motion_set == need_send_out)
                {
                    data_save.filament[read_num].motion_set = on_use;
                }
                pre_pull_count = 0;
                data_save.filament[read_num].pressure = 0x2B00;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F)) // 07 7F
            {
                data_save.filament[read_num].motion_set = on_use;
                data_save.filament[read_num].pressure = 0x2B00;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x00)) // 07 00
            {
                if (pre_pull_count < 10000 && pre_pull_count > 1000) // 10s pre pull
                {
                    if (data_save.filament[read_num].motion_set == on_use)
                    {
                        // data_save.filament[read_num].motion_set = pre_pull;
                    }
                    pre_pull_count += time_used;
                }
                else
                {
                    data_save.filament[read_num].motion_set = on_use;
                }
                data_save.filament[read_num].pressure = 0x2B00;
            }
            idle_count = 0;
        }
        else if ((read_num == 0xFF))
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00(FF)
            {
                if (data_save.BambuBus_now_filament_num < 4)
                {
                    _filament *filament = &(data_save.filament[data_save.BambuBus_now_filament_num]);
                    if (filament->motion_set == on_use || filament->motion_set == pre_pull)
                        filament->motion_set = need_pull_back;
                    filament->pressure = 0x4700;
                }
            }
            else if ((statu_flags == 0x01) && (fliment_motion_flag == 0x00)) // 01 00(FF)
            {
                for (auto i = 0; i < 4; i++)
                {
                    if (data_save.filament[i].motion_set != on_use || idle_count > 59999)
                        data_save.filament[i].motion_set = idle;
                    data_save.filament[i].pressure = 0xFFFF;
                }
                if (idle_count < 60000)
                    idle_count += time_used;
            }
        }
    }
    else if (BambuBus_address == 0x00) // none
    {
    }
    else
        return false;
    return true;
}

unsigned char Hit_res[] = {0x7D, 0x0E,
                           0x00, 0x00,             // amsnum + readnum
                           0x00, 0x00, 0x00, 0x00, // flag
                           0x00, 0x00, 0x00, 0x00, // meters
                           0x55,                   // online
                           0xE4};                  // crc8校验位
void send_for_Hit(unsigned char *buf, int length)
{
    Hit_res[0] = 0x7D;
    unsigned char AMS_num = buf[3];
    unsigned char read_num = buf[4];
    /*
    if ((buf[5] &= 0x30) && BambuBus_address == 0)
    {
        BambuBus_address = 0x700;
    }
    else if ((buf[5] &= 0xC0) && BambuBus_address == 0)
    {
        BambuBus_address = 0x1200;
    }    
    */
    if (data_save.bmcu != AMS_num)
        return;
    set_motion_res_datas(Hit_res + 2, AMS_num, read_num, 0);
    package_send_with_crc(Hit_res, sizeof(Hit_res));
}
// 3D E0 3C 12 04 00 00 00 00 09 09 09 00 00 00 00 00 00 00
// 02 00 E9 3F 14 BF 00 00 76 03 6A 03 6D 00 E5 FB 99 14 2E 19 6A 03 41 F4 C3 BE E8 01 01 01 01 00 00 00 00 64 64 64 64 0A 27
// 3D E0 2C C9 03 00 00
// 04 01 79 30 61 BE 00 00 03 00 44 00 12 00 FF FF FF FF 00 00 44 00 54 C1 F4 EE E7 01 01 01 01 00 00 00 00 FA 35
#define C_test 0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x80, 0xBF, \
               0x00, 0x00, 0x00, 0x00, \
               0x36, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x27, 0x00, \
               0x55,                   \
               0xFF, 0xFF, 0xFF, 0xFF, \
               0xFF, 0xFF, 0xFF, 0xFF,
/*#define C_test 0x00, 0x00, 0x02, 0x02, \
               0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x00, 0xC0, \
               0x36, 0x00, 0x00, 0x00, \
               0xFC, 0xFF, 0xFC, 0xFF, \
               0x00, 0x00, 0x27, 0x00, \
               0x55,                   \
               0xC1, 0xC3, 0xEC, 0xBC, \
               0x01, 0x01, 0x01, 0x01,
00 00 02 02 EB 8F CA 3F 49 48 E7 1C 97 00 E7 1B F3 FF F2 FF 00 00 90 00 75 F8 EE FC F0 B6 B8 F8 B0 00 00 00 00 FF FF FF FF*/
/*
#define C_test 0x00, 0x00, 0x02, 0x01, \
                0xF8, 0x65, 0x30, 0xBF, \
                0x00, 0x00, 0x28, 0x03, \
                0x2A, 0x03, 0x6F, 0x00, \
                0xB6, 0x04, 0xFC, 0xEC, \
                0xDF, 0xE7, 0x44, 0x00, \
                0x04, \
                0xC3, 0xF2, 0xBF, 0xBC, \
                0x01, 0x01, 0x01, 0x01,*/
unsigned char Cxx_res[] = {0x7D, 0x0E,
                           0x00, 0x00,             // amsnum + readnum
                           0x00, 0x00, 0x00, 0x00, // flag
                           0x00, 0x00, 0x00, 0x00, // meters
                           0x55,                   // online
                           0xE4};                  // crc8校验位

// 从机执行器命令层：0x03 状态帧和 0x08 命令都先翻译成这些命令。
enum ExecCmd : uint8_t
{
    EXEC_IDLE,
    EXEC_SEND,
    EXEC_SEND_RESET,
    EXEC_PULL,
    EXEC_ONUSE,
    EXEC_PREPULL,
    EXEC_STOP_ALL,
};

// 只写运行时 mode/pressure，不碰 Flash。
static bool exec_command(uint8_t tray, ExecCmd cmd)
{
    if (cmd == EXEC_STOP_ALL)
    {
        for (int i = 0; i < 4; i++)
        {
            if (data_save.filament[i].motion_set != on_use)
                data_save.filament[i].motion_set = idle;
            data_save.filament[i].pressure = 0xFFFF;
        }
        return true;
    }

    if (tray >= 4)
        return false;

    if (cmd == EXEC_SEND || cmd == EXEC_SEND_RESET)
    {
        if (cmd == EXEC_SEND_RESET && data_save.BambuBus_now_filament_num != tray)
        {
            for (int i = 0; i < 4; i++)
            {
                data_save.filament[i].motion_set = idle;
                data_save.filament[i].pressure = 0xFFFF;
            }
        }
        data_save.filament[tray].motion_set = need_send_out;
        data_save.filament[tray].pressure = 0x4700;
    }
    else if (cmd == EXEC_PULL)
    {
        data_save.filament[tray].motion_set = need_pull_back;
        data_save.filament[tray].pressure = 0x4700;
    }
    else if (cmd == EXEC_ONUSE)
    {
        data_save.filament[tray].motion_set = on_use;
        data_save.filament[tray].pressure = 0x2B00;
    }
    else if (cmd == EXEC_PREPULL)
    {
        data_save.filament[tray].motion_set = pre_pull;
        data_save.filament[tray].pressure = 0x2B00;
    }
    else
    {
        data_save.filament[tray].motion_set = idle;
        data_save.filament[tray].pressure = 0xFFFF;
    }
    data_save.BambuBus_now_filament_num = tray;
    return true;
}

// 0x03 直写模式：flag 低 4 位就是通道状态（与 0x7D 回复的 flagx 一致）。
static void write_channel_flag(uint8_t tray, uint8_t flag)
{
    if (tray >= 4)
        return;
    switch (flag & 0x0F)
    {
    case 0x01:
        data_save.filament[tray].motion_set = need_pull_back;
        data_save.filament[tray].pressure = 0x4700;
        break;
    case 0x02:
        data_save.filament[tray].motion_set = need_send_out;
        data_save.filament[tray].pressure = 0x4700;
        break;
    case 0x03:
        data_save.filament[tray].motion_set = pre_pull;
        data_save.filament[tray].pressure = 0x2B00;
        break;
    case 0x04:
        data_save.filament[tray].motion_set = on_use;
        data_save.filament[tray].pressure = 0x2B00;
        break;
    default:
        data_save.filament[tray].motion_set = idle;
        data_save.filament[tray].pressure = 0xFFFF;
        break;
    }
}

void send_for_Cxx(unsigned char *buf, int length)
{
    // 旧 10 字节 0x03 兼容格式：
    // 9D 0A 03 AMS tray flagx reserved online pull crc
    // statu_flags 位改作 flagx；fliment_motion_flag 位待用。
    // 只修改 tray 指向的当前通道，其他通道保持原状态。
    if (length < 10)
        return;

    unsigned char AMS_num = buf[3];
    unsigned char tray = buf[4];
    unsigned char flag = buf[5];
    Cxx_res[0] = 0x7D;

    if (data_save.bmcu != AMS_num)
        return;

    if (tray < 4)
    {
        write_channel_flag(tray, flag);
        data_save.BambuBus_now_filament_num = tray;
        Bmcu_select_flag = true;
    }
    else
    {
        Bmcu_select_flag = false;
    }

    Host_stu_update(buf[7], buf[8]); // online / pull
    set_motion_res_datas(Cxx_res + 2, AMS_num, tray, 0);
    package_send_with_crc(Cxx_res, sizeof(Cxx_res));
}
/*
0x00, 0x00, 0x00, 0xFF, // 0x0C...
0x00, 0x00, 0x80, 0xBF, // distance
0x00, 0x00, 0x00, 0xC0,
0x00, 0xC0, 0x5D, 0xFF,
0xFE, 0xFF, 0xFE, 0xFF, // 0xFE, 0xFF, 0xFE, 0xFF,
0x00, 0x44, 0x00, 0x00,
0x10,
0xC1, 0xC3, 0xEC, 0xBC,
0x01, 0x01, 0x01, 0x01,
*/
unsigned char Dxx_res[] = {0x7D, 0x0E,
                           0x00, 0x00,             // amsnum + readnum
                           0x00, 0x00, 0x00, 0x00, // flag
                           0x00, 0x00, 0x00, 0x00, // meters
                           0x55,                   // online
                           0xE4};                  // crc8校验位
/*unsigned char Dxx_res2[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                            0x00, 0x75, 0x01, 0x11,
                            0x0C, 0x04, 0x04, 0x03,
                            0x08, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x03, 0x03,
                            0x5F, 0x6E, 0xD7, 0xBE,
                            0x00, 0x00, 0x03, 0x00,
                            0x44, 0x00, 0x01, 0x00,
                            0xFE, 0xFF, 0xFE, 0xFF,
                            0x00, 0x00, 0x00, 0x00,
                            0x50,
                            0xC1, 0xC3, 0xED, 0xE9,
                            0x01, 0x01, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            0xFF, 0xFF, 0xFF, 0xFF,
                            0xEC, 0xF0};*/
bool need_res_for_06 = false;
uint8_t res_for_06_num = 0xFF;
int last_detect = 0;
uint8_t filament_flag_detected = 0;

void send_for_Dxx(unsigned char *buf, int length)
{
    // unsigned char filament_flag_on = 0x00;
    // unsigned char filament_flag_NFC = 0x00;
    unsigned char AMS_num = buf[3];
    unsigned char read_num = buf[4];
    unsigned char statu_flags = buf[5];
    unsigned char fliment_motion_flag = buf[6];
    Dxx_res[0] = 0x7D;
    Host_stu_update(buf[7], buf[8]); // 更新缓冲状态
    if (data_save.bmcu != AMS_num)
    {
        if (read_num != 0xFF)
        {
            Bmcu_select_flag = false;
            /*
            for (int i = 0; i < 4; i++)
            {
                if (data_save.filament[i].motion_set == need_pull_back)
                    set_filament_motion(i, idle);
            }
            */
        }
        return;
    }
    else
    {
        if (read_num < 4 && !Bmcu_select_flag)
        {
            Bmcu_select_flag = true;
            // Bmcu_selest_count = 0;
        }
        else if (read_num == 0xFF && statu_flags == 0x01 && Bmcu_select_flag)
        {
            Bmcu_select_flag = false;
            // Bmcu_selest_count = get_time64() + 15000;
        }
    }

    if (!set_motion(AMS_num, read_num, statu_flags, fliment_motion_flag))
        return;
    /*if (need_res_for_06)
    {
        Dxx_res2[1] = 0xC0 | (package_num << 3);
        Dxx_res2[9] = filament_flag_on;
        Dxx_res2[10] = filament_flag_on - filament_flag_NFC;
        Dxx_res2[11] = filament_flag_on - filament_flag_NFC;
        Dxx_res[19] = flagx;
        Dxx_res[20] = Dxx_res2[12] = res_for_06_num;
        Dxx_res2[13] = filament_flag_NFC;
        Dxx_res2[41] = get_filament_left_char();
        package_send_with_crc(Dxx_res2, sizeof(Dxx_res2));
        need_res_for_06 = false;
    }
    else*/

    set_motion_res_datas(Dxx_res + 2, AMS_num, read_num, statu_flags);
    package_send_with_crc(Dxx_res, sizeof(Dxx_res));
}
unsigned char REQx6_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x06,
                             0x00, 0x00, 0x00, 0x00,
                             0x04, 0x04, 0x04, 0xFF, // flags
                             0x00, 0x00, 0x00, 0x00,
                             C_test 0x00, 0x00, 0x00, 0x00,
                             0x64, 0x64, 0x64, 0x64,
                             0x90, 0xE4};
void send_for_REQx6(unsigned char *buf, int length)
{
    // 0x06 设置参数帧：
    // 9D 0A 06 AMS cmd channel value_lo value_hi reserved crc
    // cmd: 03=pull_ms, 04=pull_t1_ms, 05=pwm_zero(all)
    if (length < 10)
        return;

    unsigned char AMS_num = buf[3];
    if (data_save.bmcu != AMS_num)
        return;

    unsigned char cmd = buf[4];
    uint16_t value = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

    if (cmd == 0x03)
    {
        MOTOR_set_time_pull(true, value);
    }
    else if (cmd == 0x04)
    {
        MOTOR_set_time_pull(false, value);
    }
    else if (cmd == 0x05)
    {
        MOTOR_set_pwm_zero(value);
    }
}

void NFC_detect_run()
{
    /*uint64_t time = GetTick();
    return;
    if (time > last_detect + 3000)
    {
        filament_flag_detected = 0;
    }*/
}
uint8_t online_detect_num2[] = {0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, // 序列号？(额外包含之前一位)
                                0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t online_detect_num[] = {0x90, 0x31, 0x33, 0x34, 0x36, 0x35, 0x02, 0x00, 0x37, 0x39, 0x33, 0x38, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char F01_res[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
    0x16,
    0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x33, 0xF0};
void send_for_Fxx(unsigned char *buf, int length)
{
    // uint8_t AMS_num;
    uint8_t F00_res[4 * sizeof(F01_res)];
    if ((buf[5] == 0x00))
    {
        // if((buf[8]==0x30)&&(buf[9]==0x31));
        for (auto i = 0; i < 4; i++)
        {
            memcpy(F00_res + i * sizeof(F01_res), F01_res, sizeof(F01_res));
            F00_res[i * sizeof(F01_res) + 5] = 0;
            F00_res[i * sizeof(F01_res) + 6] = i;
            F00_res[i * sizeof(F01_res) + 7] = i;
        }
        // package_send_with_crc(F00_res, sizeof(F00_res));
    }

    if ((buf[5] == 0x01) && (buf[6] < 4))
    {
        memcpy(F01_res + 4, buf + 4, 3);
        // memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
        /*
        if (buf[6])
            memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
        else
            memcpy(F00_res + 8, online_detect_num2, sizeof(online_detect_num2));*/
        // memcpy(online_detect_num, buf + 8, sizeof(online_detect_num));
        //  F00_res[5] = buf[5];
        //  F00_res[6] = buf[6];
        // package_send_with_crc(F01_res, sizeof(F01_res));
    }
}
// 3D C5 0D F1 07 00 00 00 00 00 00 CE EC
// 3D C0 0D 6F 07 00 00 00 00 00 00 9A 70

unsigned char NFC_detect_res[] = {0x3D, 0xC0, 0x0D, 0x6F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xE8};
void send_for_NFC_detect(unsigned char *buf, int length)
{
    last_detect = 20;
    filament_flag_detected = 1 << buf[6];
    NFC_detect_res[6] = buf[6];
    NFC_detect_res[7] = buf[7];
    // package_send_with_crc(NFC_detect_res, sizeof(NFC_detect_res));
}

unsigned char long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void send_for_long_packge_MC_online(unsigned char *buf, int length)
{
    long_packge_data data;
    uint8_t AMS_num = printer_data_long.datas[0];
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    if (printer_data_long.target_address == 0x0700)
    {
    }
    else if (printer_data_long.target_address == 0x1200)
    {
    }
    /*else if(printer_data_long.target_address==0x0F00)
    {

    }*/
    else
    {
        return;
    }

    data.datas = long_packge_MC_online;
    data.datas[0] = AMS_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    // Bambubus_long_package_send(&data);
}
unsigned char serial_number[] = {"STUDY0ONLY"};
unsigned char long_packge_version_serial_number[] = {9, // length
                                                     'S', 'T', 'U', 'D', 'Y', 'O', 'N', 'L', 'Y', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // serial_number#2
                                                     0x30, 0x30, 0x30, 0x30,
                                                     0xFF, 0xFF, 0xFF, 0xFF,
                                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

unsigned char long_packge_version_version_and_name_AMS_lite[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                                 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_version(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    uint8_t AMS_num = printer_data_long.datas[0];
    unsigned char *long_packge_version_version_and_name;

    if (printer_data_long.target_address == 0x0700)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS08;
    }
    else if (printer_data_long.target_address == 0x1200)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS_lite;
    }
    else
    {
        return;
    }

    switch (printer_data_long.type)
    {
    case 0x402:

        AMS_num = printer_data_long.datas[33];
        long_packge_version_serial_number[0] = sizeof(serial_number);
        memcpy(long_packge_version_serial_number + 1, serial_number, sizeof(serial_number));
        data.datas = long_packge_version_serial_number;
        data.data_length = sizeof(long_packge_version_serial_number);

        data.datas[65] = AMS_num;
        break;
    case 0x103:

        AMS_num = printer_data_long.datas[0];
        data.datas = long_packge_version_version_and_name;
        data.data_length = sizeof(long_packge_version_version_and_name_AMS08);
        data.datas[20] = AMS_num;
        break;
    default:
        return;
    }

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    // Bambubus_long_package_send(&data);
}
unsigned char s = 0x01;
// unsigned char filament_res[] = {0x7D, 0x0A, 0x08,
//                                 0x00, 0x00, // amsnum + taynum
//                                 0x00, 0x00, // 公用控制位 + 专用控制位
//                                 0x00};      // crc8 校验
unsigned char Set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
const unsigned char select_bmcu_filament_name[] = "TPU-AMS";                // ID: GFU02
const unsigned char reset_bmcu_meter_color[4] = {0xFF, 0xFF, 0xFF, 0xFF};   // white
const unsigned char reset_bmcu_channel_color[4] = {0xFF, 0xF1, 0x44, 0xFF}; // 黄色
void send_for_Set_filament(unsigned char *buf, int length)
{
    uint8_t AMS_num = buf[3];
    uint8_t read_num = buf[4];
    uint8_t command_1 = buf[5];
    uint8_t command_2 = buf[6];
    if (command_1 == 0xE0)
        bmcu_reset = true;
    if (data_save.bmcu != AMS_num)
    {
        return;
    }
    if (read_num != 0xFF && read_num >= 4)
        return;
    else
    {
        if (command_2 == 0xD1) // 白色  --重置指定通道里程（仅 RAM）
            reset_filament_meters(read_num);
        else if (command_2 == 0xB9) //--指定通道need_pull_back
            exec_command(read_num, EXEC_PULL);
        else if (command_2 == 0xC9) //--指定通道need_send_out
            exec_command(read_num, EXEC_SEND);
        else if (command_2 == 0xD9) //--指定通道onuse
            exec_command(read_num, EXEC_ONUSE);
        else if (command_2 == 0xE9) //--指定通道idle
            exec_command(read_num, EXEC_IDLE);
        else if (command_2 == 0xF9) //--指定通道pre_pull
            exec_command(read_num, EXEC_PREPULL);
        else if (command_2 == 0xD3 && read_num == 0) // 棕色  --电机退料时间设定 二段
            MOTOR_set_time_pull(true, 1000);
        else if (command_2 == 0xD3 && read_num == 1) // 棕色  --电机退料时间设定
            MOTOR_set_time_pull(true, 2000);
        else if (command_2 == 0xD3 && read_num == 2) // 棕色  --电机退料时间设定 --默认 8s
            MOTOR_set_time_pull(true, 4000);
        else if (command_2 == 0xD3 && read_num == 3) // 棕色  --电机退料时间设定
            MOTOR_set_time_pull(true, 8000);
        else if (command_2 == 0xD5 && read_num == 0) // 岩石灰  --电机退料时间设定  一段
            MOTOR_set_time_pull(false, 1000);
        else if (command_2 == 0xD5 && read_num == 1) // 岩石灰  --电机退料时间设定
            MOTOR_set_time_pull(false, 2000);
        else if (command_2 == 0xD5 && read_num == 2) // 岩石灰  --电机退料时间设定 --默认 8s
            MOTOR_set_time_pull(false, 4000);
        else if (command_2 == 0xD5 && read_num == 3) // 岩石灰  --电机退料时间设定
            MOTOR_set_time_pull(false, 8000);
        else if (command_2 == 0xD7 && read_num == 0) ////灰色  --电机pwm 设定
            MOTOR_set_pwm_zero(220);
        else if (command_2 == 0xD7 && read_num == 1) ////灰色  --电机pwm 设定
            MOTOR_set_pwm_zero(300);
        else if (command_2 == 0xD7 && read_num == 2) ////灰色  --电机pwm 设定  --默认
            MOTOR_set_pwm_zero(380);
        else if (command_2 == 0xD7 && read_num == 3) ////灰色  --电机pwm 设定
            MOTOR_set_pwm_zero(460);
        // else if (command_2 == 0xD7)               ////灰色  --电机标定
        // MOTOR_get_pwm_zero();
    }
}
package_type BambuBus_run()
{
    package_type stu = BambuBus_package_NONE;
    static uint64_t time_set = 0;
    static uint64_t select_false_since = 0;
    static bool select_false_valid = false;
    static uint64_t next_settings_retry = 0;
    uint64_t timex = get_time64();
    // static bool save_s = false;
    /*for (auto i : data_save.filament)
    {
        i->motion_set = idle;
    }*/

    if (BambuBus_have_data)
    {
        int data_length = BambuBus_have_data;
        stu = get_packge_type(buf_X, data_length); // have_data
        delay(1);
        BambuBus_have_data = 0;
        switch (stu)
        {
        case BambuBus_package_heartbeat:
            send_for_Hit(buf_X, data_length);
            time_set = timex + 2000;
            break;
        case BambuBus_package_filament_motion_short:
            send_for_Cxx(buf_X, data_length);
            break;
        case BambuBus_package_filament_motion_long:
            send_for_Dxx(buf_X, data_length);
            break;
        case BambuBus_package_set_filament:
            send_for_Set_filament(buf_X, data_length);
            break;
        case BambuBus_package_REQx6:
            send_for_REQx6(buf_X, data_length);
            break;
        default:
            break;
        }
    }
    if (timex > time_set)
    {
        stu = BambuBus_package_ERROR; // offline
    }
    // 只保存设置参数：settings dirty 后，等打印机空闲满 10 分钟追加一次；
    // 里程/材料属性/打印结束都不触发保存。
    if (Bmcu_select_flag)
    {
        select_false_since = 0;
        select_false_valid = false;
    }
    else if (!select_false_valid)
    {
        select_false_since = timex;
        select_false_valid = true;
    }

    const bool idle_10min =
        (!Bmcu_select_flag && select_false_valid && (timex - select_false_since) >= 600000);
    if (Settings_is_dirty() && idle_10min && timex >= next_settings_retry)
    {
        if (!Settings_save())
            next_settings_retry = timex + 10000;
    }

    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);
    /*
    if (time_check < timex - 180000 && time_check != 0)
    {
        if (!Bmcu_select_flag)
        {
            time_check = 0;
        }
        else if (Bmcu_select_flag && !filament_check())
        {
            Bmcu_select_flag = false;
        }
    }
    else if (Bmcu_select_flag && filament_check())
    {
        time_check = timex;
    }
    */

    // NFC_detect_run();
    return stu;
}

bool Bmcu_reset()
{
    return bmcu_reset;
}
void Bmcu_set_no_reset()
{
    bmcu_reset = false;
}
void Bmcu_set_num(uint8_t num)
{
    data_save.bmcu = num;
    Settings_get().bmcu = num;
    Settings_mark_dirty();
}
bool Bmcu_select()
{
    // return Bmcu_select_flag;

    if (Bmcu_select_flag)
    {
        Bmcu_selest_count = get_time64() + 2000;
        return true;
    }
    else if (Bmcu_selest_count < get_time64() && !Bmcu_select_flag)
        return false;
    else
        return true;
}