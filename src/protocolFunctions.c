#include "hbprotocol/protocol.h"
#include <stdio.h>
#include <string.h>

extern unsigned long millis(void);

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
// Variable & Functions for 0x27 Ping
// Sends back the received Message as a readresponse. (Sender can send a timestamp to measure latency)
unsigned long latency = 0;

void fn_customPing ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    // Keep functionality to reply to pings.
    fn_ping( s, param, cmd, msg);

    switch (cmd) {

        case PROTOCOL_CMD_READVALRESPONSE:
        {
            if(msg) {
                unsigned long timeSent;
                memcpy( &timeSent, msg->content, sizeof(timeSent));

                latency = millis() - timeSent;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x__ Paddelparameters

PARAMSTAT paddelParameters =
    { 0x70, "PaddelParametes", NULL, UI_NONE, NULL, 0, fn_defaultProcessing };

////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol( PROTOCOL_STAT *s ) {

    int errors = 0;

    errors += setParamVariable( s, 0x08, UI_NONE, (void *)&electrical_measurements, sizeof(PROTOCOL_ELECTRICAL_PARAMS));
    setParamHandler(s, 0x08, fn_defaultProcessing);

    errors += setParamVariable( s, 0x02, UI_NONE, (void *)&HallData,                sizeof(HallData));
    setParamHandler(s, 0x02, fn_defaultProcessing);

    errors += setParamVariable( s, 0x03, UI_NONE, (void *)&SpeedData,                sizeof(SpeedData));
    setParamHandler(s, 0x03, fn_defaultProcessing);

    setParamHandler(s, 0x27, fn_customPing);

    setParam(s, &paddelParameters);

    return errors;

}
