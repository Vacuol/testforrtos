#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//错误码以及对应设备顺序
enum errorList
{
    DBUSTOE = 0,
    YawGimbalMotorTOE,
    PitchGimbalMotorTOE,
    TriggerMotorTOE,
    ChassisMotor1TOE,
    ChassisMotor2TOE,
    ChassisMotor3TOE,
    ChassisMotor4TOE,

    errorListLength,
};

extern void detect_task(void const * argument);

#endif
