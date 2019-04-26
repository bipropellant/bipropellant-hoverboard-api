#pragma once

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