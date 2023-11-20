/*
 * serial_protocol.c
 *
 *  Created on: Oct. 22, 2023
 *      Author: yifeiliu
 */


#include "serial_protocol.h"

bool is_packet_valid(Packet const *p)
{
    if (p->start_byte != 0xFF || p->control_byte_0==0xFF || p->control_byte_1==0xFF)
    {
        return false;
    }
    else
    {
        uint8_t sum = 0;
        for (int i = 0; i < sizeof(Packet) - 1; ++i)
        {
            sum += ((uint8_t const *)p)[i];
        }
        return sum == p->check_sum;
    }
}
