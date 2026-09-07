#include "system_status.h"

system_status_t sys_stat;

system_status_t get_system_status()
{
    return sys_stat;
}

void set_system_status(system_status_t _sys_stat)
{
    sys_stat = _sys_stat;
    return; 
}