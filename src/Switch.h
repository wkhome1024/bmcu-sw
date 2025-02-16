#pragma once

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
    extern void Switch_init();
    extern bool Switch_read();
    extern bool get_bmcu_selected();
    extern void set_bmcu_selected(uint8_t selected);
    extern uint8_t get_filament_map_to(uint8_t num);
    extern bool check_bmcu_num(uint8_t bmcu);
    extern uint8_t get_current_bmcu_num();
    extern std::pair<uint8_t, uint8_t> get_bmcu_and_channel(uint8_t num);
    extern bool Switch_set_filament(unsigned char *buf, int length, uint8_t AMS_num, uint8_t read_num);
    extern void Switch_set_need_to_save();
    extern bool Switch_need_to_save();
    extern void Switch_save();
    extern bool Switch_need_to_delay();
    extern void Switch_set_not_to_delay();
    extern void Switch_set_need_to_delay();
    extern void Switch_set_autoready();
    extern void Switch_set_not_autoready();
    extern bool Switch_autoready();    
#ifdef __cplusplus
}
#endif