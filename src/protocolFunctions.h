#pragma once

#include "protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
extern int setup_protocol();



extern volatile PROTOCOL_ELECTRICAL_PARAMS electrical_measurements;


extern uint32_t (*HAL_GetTick)(void);
extern void (*HAL_Delay)(uint32_t Delay);
extern void (*HAL_NVIC_SystemReset)(void);
extern volatile PROTOCOL_HALL_DATA_STRUCT HallData[2];

#ifdef __cplusplus
}
#endif