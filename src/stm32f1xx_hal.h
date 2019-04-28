#pragma once

#include <stdint.h>

extern uint32_t (*HAL_GetTick)(void);
extern void (*HAL_Delay)(uint32_t Delay);
extern void (*HAL_NVIC_SystemReset)(void);
