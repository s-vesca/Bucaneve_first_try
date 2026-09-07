#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

typedef enum system_status_e
{
    SYSTEM_STATUS_IDLE = 0,
    SYSTEM_STATUS_RUNNING,
    SYSTEM_STATUS_ERROR,
    SYSTEM_STATUS_MAX_NUM
}system_status_t;

system_status_t get_system_status();
void set_system_status(system_status_t _sys_stat);

#endif //SYSTEM_STATUS_H