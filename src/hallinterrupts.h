#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////
// this is the Hall data we gather, and can be read elsewhere
// one for each wheel
typedef struct tag_HALL_DATA_STRUCT{
    long HallPosn; // 90 per revolution
    long HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    long HallPosn_lastread; // posn offset set via protocol in raw value
    long HallPosn_mm; // posn in mm
    long HallPosn_mm_lastread; // posn offset set via protocol in mm
    long HallSpeed_mm_per_s; // speed in m/s

    unsigned long HallTimeDiff;
    unsigned long HallSkipped;
} HALL_DATA_STRUCT;
extern volatile HALL_DATA_STRUCT HallData[2];

typedef struct tag_HALL_PARAMS{
    uint8_t hall_u;
    uint8_t hall_v;
    uint8_t hall_w;

    uint8_t hall;
    uint8_t last_hall;

    long long hall_time;
    long long last_hall_time;
    unsigned int timerwraps;
    unsigned int last_timerwraps;

    int incr;

    int zerospeedtimeout;

    // contant - modifies sign of calculated values
    int direction;


    int dmacount;
} HALL_PARAMS;

extern volatile HALL_PARAMS local_hall_params[2];

#ifdef __cplusplus
}
#endif