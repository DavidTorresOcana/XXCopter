
#ifndef MAVLINK_BRIDGE_HEADER_H_
#define MAVLINK_BRIDGE_HEADER_H_

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"
#include "rs232.h"

mavlink_system_t mavlink_system;
static int comport=22;

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{

    if (chan == MAVLINK_COMM_0)
    {
    	RS232_SendByte(comport,ch);
    }
}


#endif /* MAVLINK_BRIDGE_HEADER_H_ */
