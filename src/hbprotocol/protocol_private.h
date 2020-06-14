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




/////////////////////////////////////////////////////////////////
// processes ASCII characters
void ascii_byte(PROTOCOL_STAT *s, unsigned char byte );

/////////////////////////////////////////////////////////////////
// processes machine protocol messages
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG3full *msg);

/////////////////////////////////////////////////////////////////
// get buffer level
int mpTxQueued(MACHINE_PROTOCOL_TX_BUFFER *buf);

/////////////////////////////////////////////////////////////////
// callback which can be used for "debugging"
extern void (*debugprint)(const char str[]);

// get param function handler
PARAMSTAT_FN getParamHandler( PROTOCOL_STAT *s, unsigned char code );


#ifdef __cplusplus
}
#endif
