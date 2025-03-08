#include "switch.h"
#include "BambuBus.h"

#define BMCUSwitch_version 3
#define use_flash_addr ((uint32_t)0x0800FA00)
struct alignas(4) switch_save_struct
{
    uint32_t version = BMCUSwitch_version;
    uint8_t bmcu_num = 0;
    unsigned char current_bmcu_num = 0;
    unsigned char filament_map_to[4];
} switch_save;

const unsigned char select_bmcu_filament_name[] = "TPU-AMS"; //ID: GFU02
const unsigned char set_bmcu_num_color[4] = {0xFF, 0xFF, 0xFF, 0xFF}; //white
const unsigned char set_bmcu_auto_color[4] = {0xD3, 0xC5, 0xA3, 0xFF};//沙漠黄
const unsigned char haset_bmcu_channel_color[4] = {0x40, 0x61, 0x00, 0xFF};
const unsigned char haceck_bmcu_channel_color[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
const unsigned char reset_bmcu_channel_color[4] = {0xFF, 0xF1, 0x44, 0xFF}; //黄色
const unsigned char set_bmcu_filament_color0[4] = {0xAF, 0x79, 0x33, 0xFF}; //棕色
const unsigned char set_bmcu_filament_color1[4] = {0x89, 0x89, 0x89, 0xFF}; //岩石灰
const unsigned char set_bmcu_filament_color2[4] = {0xBC, 0xBC, 0xBC, 0xFF}; //灰色
const unsigned char set_bmcu_filament_color3[4] = {0x16, 0x16, 0x16, 0xFF}; //黑色
void Switch_init()
{
    bool _init_ready = Switch_read();
    if (!_init_ready)
    {
        switch_save.bmcu_num = 0;
        switch_save.current_bmcu_num = 0;
        switch_save.filament_map_to[0] = 0;
        switch_save.filament_map_to[1] = 1;
        switch_save.filament_map_to[2] = 2;
        switch_save.filament_map_to[3] = 3;
        //Switch_save();
    }
}
bool Switch_read()
{
    switch_save_struct *ptr = (switch_save_struct *)(use_flash_addr);
    if (ptr->version == BMCUSwitch_version)
    {
        memcpy(&switch_save, ptr, sizeof(switch_save));
        return true;
    }
    return false;
}

uint8_t get_filament_map_to(uint8_t num)
{
    return switch_save.filament_map_to[num];
}

bool get_bmcu_selected()
{
    return switch_save.bmcu_num == switch_save.current_bmcu_num;
}

bool check_bmcu_num(uint8_t bmcu)
{
    return switch_save.bmcu_num == bmcu;
}

std::pair<uint8_t, uint8_t> get_bmcu_and_channel(uint8_t num) {
    uint8_t number = get_filament_map_to(num);
    uint8_t bmcuNumber = number / 4;      // 计算 AMS 编号
    uint8_t channelNumber = number % 4; // 计算通道编号
    return {bmcuNumber, channelNumber};
}

void set_bmcu_selected(uint8_t selected)
{
    switch_save.current_bmcu_num = selected;
    //Switch_set_need_to_save();
}
uint8_t get_current_bmcu_num()
{
    return switch_save.current_bmcu_num;
}

bool Switch_set_filament(unsigned char *buf, int length, uint8_t AMS_num, uint8_t read_num)
{
    if (memcmp(select_bmcu_filament_name, buf + 23, sizeof(select_bmcu_filament_name)) == 0)
    { 
        if (memcmp(buf + 15, set_bmcu_filament_color0, 4) == 0)
        {
            switch_save.filament_map_to[0] = switch_save.current_bmcu_num * 4 + read_num;
        }
        else if(memcmp(buf + 15, set_bmcu_filament_color1, 4) == 0)
        {
            switch_save.filament_map_to[1] = switch_save.current_bmcu_num * 4 + read_num;
        }
        else if(memcmp(buf + 15, set_bmcu_filament_color2, 4) == 0)
        {
            switch_save.filament_map_to[2] = switch_save.current_bmcu_num * 4 + read_num;
        }
        else if(memcmp(buf + 15, set_bmcu_filament_color3, 4) == 0)
        {
            switch_save.filament_map_to[3] = switch_save.current_bmcu_num * 4 + read_num;
        }
        else if(memcmp(buf + 15, haset_bmcu_channel_color, 2) == 0)
        {
            for (int i = 0; i < 16; i++)
            {
                if (memcmp(buf + 17, haceck_bmcu_channel_color + i, 1) == 0)
                {
                    switch_save.filament_map_to[read_num] = i;
                }
                
            }
            Switch_set_longpull();
        }
        else if(memcmp(buf + 15, set_bmcu_auto_color, 4) == 0)
        {
            Switch_set_autoready();
        }
        // white
        else if (
            get_bmcu_selected() 
            && memcmp(buf + 15, set_bmcu_num_color, sizeof(set_bmcu_num_color)) == 0)
        {
            switch_save.bmcu_num = read_num;
            Switch_set_not_longpull();
        }
        else if(memcmp(buf + 15, reset_bmcu_channel_color, 4) == 0)
        {
            switch_save.current_bmcu_num = read_num;
            switch_save.filament_map_to[0] = read_num * 4;
            switch_save.filament_map_to[1] = read_num * 4 + 1;
            switch_save.filament_map_to[2] = read_num * 4 + 2;
            switch_save.filament_map_to[3] = read_num * 4 + 3;
            Switch_set_need_to_delay();
            Switch_set_not_autoready();
            Switch_set_longpull();
        }
        // other color
        else if (memcmp(buf + 15, set_bmcu_num_color, sizeof(set_bmcu_num_color)) != 0)
        {
            switch_save.current_bmcu_num = read_num;
            Switch_set_need_to_delay();
            Switch_set_not_autoready();
            Switch_set_longpull();
        }

        Switch_set_need_to_save();
        return false;
    }
    if (!get_bmcu_selected())
        return false;
    return true;
}
bool switch_need_to_save = false;
void Switch_set_need_to_save()
{
    switch_need_to_save = true;
}
void Switch_save()
{
    //Flash_saves(&switch_save, sizeof(switch_save), use_flash_addr + sizeof(switch_save));
    Flash_saves(&switch_save, sizeof(switch_save), use_flash_addr);
    switch_need_to_save = false;
}
bool Switch_need_to_save()
{
    return switch_need_to_save;
}

bool switch_need_to_delay = false;
void Switch_set_need_to_delay()
{
    switch_need_to_delay = true;
}
void Switch_set_not_to_delay()
{
    switch_need_to_delay = false;
}
bool Switch_need_to_delay()
{
    return switch_need_to_delay;
}

bool switch_autoready = false;
void Switch_set_autoready()
{
    switch_autoready = true;
}
void Switch_set_not_autoready()
{
    switch_autoready = false;
}
bool Switch_autoready()
{
    return switch_autoready;
}
bool switch_longpull = false;
bool pull_check = false;
bool Switch_longpull()
{
    return switch_longpull;
}
void Switch_set_longpull()
{
    switch_longpull = true;
    pull_check = true;
}
void Switch_set_not_longpull()
{
    switch_longpull = false;
}
bool Switch_pullcheck()
{
    return pull_check;
}
void Switch_set_no_pullcheck()
{
    pull_check = false;
}