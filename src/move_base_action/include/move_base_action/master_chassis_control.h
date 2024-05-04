#ifndef __MASTER_CHASSIS_CONTROL_H__
#define __MASTER_CHASSIS_CONTROL_H__

typedef enum
{
    Unknown,
    TakeOff,
    Land
}UAV_State_e;

typedef enum
{
    Prepare_Over_Obstacles,
    Above_Obstacles,
    Flown_Over_Obstacles
}UAV_Fight_Schedule_e;

#endif