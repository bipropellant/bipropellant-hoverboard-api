#pragma once

#include "hbprotocol/protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
extern int setup_protocol();



extern volatile PROTOCOL_ELECTRICAL_PARAMS electrical_measurements;
extern volatile PROTOCOL_HALL_DATA_STRUCT HallData[2];
extern volatile PROTOCOL_SPEED_DATA SpeedData;

#ifdef __cplusplus
}
#endif
