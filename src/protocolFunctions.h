#pragma once

#include "hbprotocol/protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
extern int setup_protocol(PROTOCOL_STAT *s);
extern unsigned long latency;



extern volatile PROTOCOL_ELECTRICAL_PARAMS electrical_measurements;
extern volatile PROTOCOL_HALL_DATA_STRUCT HallData[2];
extern volatile PROTOCOL_SPEED_DATA SpeedData;


void fn_SubscribeData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );


#ifdef __cplusplus
}
#endif
