#include "protocol.h"
#include <stdio.h>

uint32_t noTick(void) { return 0; };
uint32_t (*HAL_GetTick)(void) = noTick;

void noDelay(uint32_t Delay) {};
void (*HAL_Delay)(uint32_t Delay) = noDelay;

void noReset(void) {};
void (*HAL_NVIC_SystemReset)(void) = noReset;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x02 hall data

volatile PROTOCOL_HALL_DATA_STRUCT HallData[2];


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x08 electrical_measurements

volatile PROTOCOL_ELECTRICAL_PARAMS electrical_measurements;


////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol() {

    int errors = 0;

        errors += setParamVariable( 0x08, UI_NONE, (void *)&electrical_measurements, sizeof(PROTOCOL_ELECTRICAL_PARAMS), PARAM_RW);
        setParamHandler(0x08, NULL);

        errors += setParamVariable( 0x02, UI_NONE, (void *)&HallData,                sizeof(HallData), PARAM_RW);
        setParamHandler(0x02, NULL);

    return errors;

}
