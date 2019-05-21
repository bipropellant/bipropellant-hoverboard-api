#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t (*HAL_GetTick)(void);
extern void (*HAL_Delay)(uint32_t Delay);
extern void (*HAL_NVIC_SystemReset)(void);
extern void (*consoleLog)(const char str[]);

#ifdef __cplusplus
}
#endif