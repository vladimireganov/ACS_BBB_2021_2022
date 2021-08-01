#ifndef SERIAL_SEND
#define SERIAL_SEND

#include <stdio.h>
#include <unistd.h> // read / write
#include <stdlib.h>	//one of these two is for memcpy
#include <string.h>
#include <stdint.h>

#include <rc/time.h> // for nanos

#include <serial_tools.h>
#include <settings.h>

// Below for PRId64
#include <inttypes.h>

#include "fallback_packet.h"
#include <crc16.h>
#include <tools.h>

extern fallback_packet_t serialMsg;
extern int serial_portID;

int serial_init();
int serial_getData();


typedef struct send_serial_packet_t
{
    uint32_t time_ms;
    flight_status_t flight_state;  ///< Use this channel to send current flight status
} send_serial_packet_t;

typedef struct send_serial_t
{
    uint64_t time_ns;  ///< last time when data was sent
} send_serial_t;

extern send_serial_t send_serial;
extern send_serial_packet_t send_serial_packet;

#define SEND_NUM_FRAMING_BYTES 4  // 2 START bytes + 2 Fletcher-16 checksum bytes
#define SEND_DATA_LENGTH sizeof(send_serial_packet_t)  // Actual Packet Being Sent
#define SEND_PACKET_LENGTH SEND_DATA_LENGTH + SEND_NUM_FRAMING_BYTES
#define SEND_START_BYTE0 0x81
#define SEND_START_BYTE1 0xA1

/**
 * @brief      This is the main function which needs to be marched
 *              to send data through serial
 *
 * @return     0 on success, -1 on failure
 */
int send_serial_data(void);

#endif  // SERIAL_SEND