#pragma once

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))


#define SKIP_ELECTRICAL_MEASUREMENTS   // Needed for hbprotocol module when electrical measurements not implemented here
#define EXCLUDE_DEADRECKONER           // Needed for hbprotocol moduke when deadreckoner is not implemted
#define SKIP_STM32SPECIFIC


// hbprotocol module has different way to name
#define INCLUDE_PROTOCOL2 2

#ifdef OUTPUT_PROTOCOL
  #define INCLUDE_PROTOCOL 2
  #define HALL_INTERRUPTS
#endif

#define SPEED_COEFFICIENT   0.5  // higher value == stronger. 0.0 to ~2.0?
#define STEER_COEFFICIENT   0.5  // higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#define DELAY_IN_MAIN_LOOP 5        // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.