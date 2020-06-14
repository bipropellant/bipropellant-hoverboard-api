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
#include "protocol.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


///////////////////////////////////////////////////
// used in machine_protocol.c
void ascii_byte(PROTOCOL_STAT *s, unsigned char byte );


typedef struct ASCII_FUNCTION_tag {
    int (*fn)(PROTOCOL_STAT *s, char *line, char *ascii_out);
    char *description;
} ASCII_FUNCTION;

typedef struct ASCII_IMMEDIATE_FUNCTION_tag {
    int (*fn)(PROTOCOL_STAT *s, char byte,  char *ascii_out);
    char *description;
} ASCII_IMMEDIATE_FUNCTION;

static ASCII_IMMEDIATE_FUNCTION immediate_functions[256];
static ASCII_FUNCTION line_functions[256];


static char password[] = "unlockASCII";   // unlock password, has to start with an 'u'
static int line_unlock_ascii(PROTOCOL_STAT *s, char *cmd, char *ascii_out) {
//case 'u':
    if (strlen(cmd) < strlen(password)){
        sprintf(ascii_out, "Wrong Password - Enter unlockASCII to enable ASCII input mode.\r\n");
    } else {
        for (int i = 0; i < strlen(password); i++){
            if(cmd[i] != password[i]) {
            sprintf(ascii_out, "Wrong Password - Enter unlockASCII to enable ASCII input mode.\r\n");
                break;
            }
            if(i == 10) {
                s->ascii.asciiProtocolUnlocked = 1;
                sprintf(ascii_out, "ASCII input active. Type ? for help\r\n");
            }
        }
    }
    return 1;
}

static int line_lock_ascii(PROTOCOL_STAT *s, char *cmd, char *ascii_out) {
//case 'L':
    s->ascii.asciiProtocolUnlocked = 0;
    sprintf(ascii_out, "ASCII protocol locked.\r\n");
    return 1;
}


void ascii_add_immediate( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char byte,  char *ascii_out), char* description ) {
    immediate_functions[letter].fn = fn;
    immediate_functions[letter].description = description;
    if ((letter >= 'A') && (letter <= 'Z')) {
        letter |= 0x20;
        immediate_functions[letter].fn = fn;
        immediate_functions[letter].description = NULL;
    }
}

void ascii_add_line_fn( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char *line, char *ascii_out), char *description ) {
    line_functions[letter].fn = fn;
    line_functions[letter].description = description;
    if ((letter >= 'A') && (letter <= 'Z')) {
        letter |= 0x20;
        line_functions[letter].fn = fn;
        line_functions[letter].description = NULL;
    }
}

void ascii_byte(PROTOCOL_STAT *s, unsigned char byte ){
    int skipchar = 0;
    // only if no characters buffered, process single keystorkes
    if (s->ascii.enable_immediate && (s->ascii.ascii_posn == 0)){
        // returns 1 if char should not be kept in command buffer
        if (immediate_functions[byte].fn){
            skipchar = immediate_functions[byte].fn(s, byte, s->ascii.ascii_out);
        } else {
            skipchar = 0;
        }
    }

    if (!skipchar){
        // on CR or LF, process gathered messages
        if ((byte == '\r') || (byte == '\n')){
            s->send_serial_data((unsigned char *) &byte, 1);


            if (s->ascii.ascii_posn > 0) {
                s->ascii.ascii_cmd[s->ascii.ascii_posn] = 0;
                sprintf(s->ascii.ascii_out, "- %s\r\n", s->ascii.ascii_cmd);
                s->send_serial_data((unsigned char *) s->ascii.ascii_out, strlen(s->ascii.ascii_out));
                s->ascii.ascii_out[0] = 0;

                if (!s->ascii.asciiProtocolUnlocked && (s->ascii.ascii_cmd[0] != 'u')) {
                    sprintf(s->ascii.ascii_out, "Locked. Enter unlockASCII to enable ASCII input mode.\r\n>");
                    s->send_serial_data((unsigned char *) s->ascii.ascii_out, strlen(s->ascii.ascii_out));
                    s->ascii.ascii_cmd[0] = 0;
                    s->ascii.ascii_posn = 0;
                    return;
                }

                if (line_functions[(unsigned char)s->ascii.ascii_cmd[0]].fn){
                    line_functions[(unsigned char)s->ascii.ascii_cmd[0]].fn(s, s->ascii.ascii_cmd, s->ascii.ascii_out);
                } else {
                    sprintf(s->ascii.ascii_out, "No cmd %c - use ? for help\r\n", s->ascii.ascii_cmd[0]);
                }
                s->ascii.ascii_cmd[0] = 0;
                s->ascii.ascii_posn = 0;
                // send prompt
                byte = '>';
            } else {
                byte = '>';
            }
        } else {
            if (s->ascii.ascii_posn < 20){
                s->ascii.ascii_cmd[s->ascii.ascii_posn++] = byte;
            } else {
                //byte = '#';
            }
        }
    } else {
        // no echo for immediate.
        // send prompt after immediate
        byte = '>';
    }
    if (s->ascii.ascii_out[0]){
        s->send_serial_data((unsigned char *) s->ascii.ascii_out, strlen(s->ascii.ascii_out));
        s->ascii.ascii_out[0] = 0;
    }

    // echo or prompt after processing
    if (byte) {
        s->send_serial_data((unsigned char *) &byte, 1);
    }
}


static int line_help(PROTOCOL_STAT *s, char *cmd, char *ascii_out) {
    char out[512];
    out[0] = 0;
    for (int i = 0; i < 256; i++) {
        if (line_functions[i].description) {
            int count = strlen(line_functions[i].description);
            char *p = line_functions[i].description;
            sprintf(out, "%c - ", (char)i);
            s->send_serial_data_wait((unsigned char *) out, strlen(out));
            while(count > 0) {
                int len = count;
                if (len > 128){
                    len = 128;
                }
                s->send_serial_data_wait((unsigned char *) p, len);
                p += len;
                count -= len;
            }
            strncpy(out, "\r\n", sizeof(out)-1);
            s->send_serial_data_wait((unsigned char *) out, strlen(out));
        }
    }
    ascii_out[0] = 0;
    return 1;
}



int ascii_init(PROTOCOL_STAT *s) {
    if (!s->ascii.initialised) {
        ascii_add_line_fn( 'L', line_lock_ascii, "Lock ascii protocol");
        ascii_add_line_fn( 'u', line_unlock_ascii, "unlockASCII - unlock ASCII protocol");

        ascii_add_line_fn( '?', line_help, "display help");
        s->ascii.initialised = 1;
        return 1;
    }
    return 0;
}
