#include "hbprotocol/protocol.h"
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x02 hall data

volatile PROTOCOL_HALL_DATA_STRUCT HallData[2];


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x08 electrical_measurements

volatile PROTOCOL_ELECTRICAL_PARAMS electrical_measurements;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x03 speed data

volatile PROTOCOL_SPEED_DATA SpeedData;


////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol( PROTOCOL_STAT *s ) {

    int errors = 0;

    errors += setParamVariable( s, 0x08, UI_NONE, (void *)&electrical_measurements, sizeof(PROTOCOL_ELECTRICAL_PARAMS));
    setParamHandler(s, 0x08, NULL);

    errors += setParamVariable( s, 0x02, UI_NONE, (void *)&HallData,                sizeof(HallData));
    setParamHandler(s, 0x02, NULL);

    errors += setParamVariable( s, 0x03, UI_NONE, (void *)&SpeedData,                sizeof(SpeedData));
    setParamHandler(s, 0x03, NULL);

    return errors;

}
