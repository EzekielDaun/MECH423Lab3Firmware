/*
 * serial_protocol.h
 *
 *  Created on: Oct. 22, 2023
 *      Author: yifeiliu
 */

#ifndef SERIAL_PROTOCOL_H_
#define SERIAL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t start_byte;
    uint8_t control_byte_0;
    uint8_t control_byte_1;
    uint8_t data_byte_0;
    uint8_t data_byte_1;
    uint8_t data_byte_2;
    uint8_t data_byte_3;
    uint8_t data_byte_4;
    uint8_t data_byte_5;
    uint8_t data_byte_6;
    uint8_t data_byte_7;
    uint8_t data_byte_8;
    uint8_t data_byte_9;
    uint8_t data_byte_10;
    uint8_t data_byte_11;
    uint8_t check_sum;
} Packet;

bool is_packet_valid(Packet const *p);

#endif /* SERIAL_PROTOCOL_H_ */
