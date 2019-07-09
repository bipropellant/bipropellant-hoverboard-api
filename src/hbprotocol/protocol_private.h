/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <stdint.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif


/////////////////////////////////////////////////////////////////
// 'machine' protocol structures and definitions
//
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
/////////////////////////////////////////////////////////////////


#pragma pack(push, 1)
typedef struct tag_PROTOCOL_LEN_ONWARDS {
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[sizeof( ((PROTOCOL_MSG2 *)0)->bytes )];  // variable number of data bytes, with a checksum on the end, cmd is first
} PROTOCOL_LEN_ONWARDS;
#pragma pack(pop)

// content of 'bytes' above, for single byte commands
#pragma pack(push, 1)
typedef struct tag_PROTOCOL_BYTES {
    unsigned char cmd; //
    unsigned char bytes[sizeof( ((PROTOCOL_MSG2 *)0)->bytes ) - sizeof(unsigned char)]; // cmd is part of bytes and needs to be substracted
} PROTOCOL_BYTES;
#pragma pack(pop)



// content of 'bytes' above, for single byte commands
#pragma pack(push, 1)
typedef struct tag_PROTOCOL_BYTES_READVALS {
    unsigned char cmd; // 'R'
    unsigned char code; // code of value to read
} PROTOCOL_BYTES_READVALS;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct tag_PROTOCOL_BYTES_WRITEVALS {
    unsigned char cmd; // 'W'
    unsigned char code; // code of value to write
    unsigned char content[sizeof( ((PROTOCOL_MSG2 *)0)->bytes ) - sizeof(unsigned char) - sizeof(unsigned char)]; // cmd and code are part of bytes and need to be substracted
} PROTOCOL_BYTES_WRITEVALS;
#pragma pack(pop)





/////////////////////////////////////////////////////////////////
// processes ASCII characters
void ascii_byte(PROTOCOL_STAT *s, unsigned char byte );

/////////////////////////////////////////////////////////////////
// processes machine protocol messages
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);

/////////////////////////////////////////////////////////////////
// get buffer level
int mpTxQueued(MACHINE_PROTOCOL_TX_BUFFER *buf);

/////////////////////////////////////////////////////////////////
// callback which can be used for "debugging"
extern void (*debugprint)(const char str[]);

// get param function handler
PARAMSTAT_FN getParamHandler(unsigned char code);


#ifdef __cplusplus
}
#endif
