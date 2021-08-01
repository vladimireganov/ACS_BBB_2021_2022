// Updated code (Feb 2019)
//
// Using data structure for packets with:
// Two start bytes:  0x81, 0xA1
// [Not included: Message ID (one byte), Message payload size (one byte) since we only have one message type]
// Message data (xbee_packet_t length)
// Fletcher-16 checksum (two bytes) computed starting with Message payload size
//
// Note:  This MBin protocol is commonly used on embedded serial devices subject to errors

#include <serial_comms.h>

int serial_portID;  // Defined as extern in xbee_packet_t.h

send_serial_packet_t send_serial_packet;
send_serial_t send_serial;

// Information local to this file
void readRingBuffer();
#define RING_BUFSIZE 256*2
#define RING_INC(a)(a<(RING_BUFSIZE-1))?(a+1):0  // Increment ring buffer index macro
int ring_overflow=0, rdIndex=0, wrIndex=0;
unsigned char ringbuffer[RING_BUFSIZE];

int serial_init() {
  int baudRate  = settings.serial_port_1_baud;
  char port[20];
  strcpy(port, settings.serial_port_1);
  serial_portID = serial_open(port,baudRate,0); // Nonblocking = 0, BLOCKING = 1
  printf(" Serial port %s \n", port);
  printf(" Baud_rate %d \n", baudRate);
  if(serial_portID == -1)    {
    printf("Failed to open Serial Port\n");
    return -1;
  }
  return 0;
}


// Read message received from XBee; use ring buffer to assure no data loss
int serial_getData()
{
  int k;
  unsigned char buffer;
  // Populate ring buffer
  for (k=0;k<RING_BUFSIZE;k++) {
    if (ring_overflow) { // Overflow condition on last read attempt
      if (rdIndex == wrIndex) { // Indices equal:  overflow still in effect
	return -1;  // Do not read data; reading data would cause overflow
	break;
      } else
	ring_overflow = 0;  // Reset overflow flag
    }

    if (read(serial_portID, &buffer, 1) > 0) { // Read from buffer: returns 0 or -1 if no data
      ringbuffer[wrIndex] = buffer;
	  wrIndex = RING_INC(wrIndex);
      if (wrIndex == rdIndex) ring_overflow = 1;
    } else
      break;
  }

  // Read and process data in ring buffer; print data
  readRingBuffer();
  return 0;
}



// readRingBuffer()
#define startByte1 0x81
#define startByte2 0xA1

//uint64_t last_time;

void readRingBuffer()
{
  static unsigned char msgState = 0, msglength = 0;
  static unsigned char msgdata[SERIAL_DATA_LENGTH], ck0, ck1;
  //printf("\n Trying to read data \n");

  while(ring_overflow || (rdIndex != wrIndex)) { //Don't get ahead of the receiving data 
    if (ring_overflow) ring_overflow = 0; // Reset buffer overflow flag
    // Case 0:  Current character is first message header byte
    if((ringbuffer[rdIndex] == startByte1) && !msgState) {
	    //printf("\n Received Start byte!\n"); //test if xbee is working and receiving start byte
        msgState = 1; 
        msglength = 0;
    }
     
    // Case 1:  Current character is second message header byte
    else if (msgState == 1) {
      if (ringbuffer[rdIndex] == startByte2) msgState = 2; 
      else msgState = 0;  // Bad message
      ck0 = 0;  // Initialize Fletcher checksum bytes
      ck1 = 0;
      msglength = 0;  // Initialize number of message payload data bytes read thusfar
    }
       
    // Case 2:  Read data bytes into msgdata[] array
    else if (msgState == 2) {
      msgdata[msglength++] = ringbuffer[rdIndex];
      ck0 += ringbuffer[rdIndex];
      ck1 += ck0;

      //printf("\n byte = %X", ringbuffer[rdIndex]);
      if (msglength == SERIAL_DATA_LENGTH) msgState = 3; // Done reading data
    }
    
    // Case 3:  Read & check the first checksum byte
    // Throw away data if full checksum doesn't match
    else if (msgState == 3) {
      if (ck0 != ringbuffer[rdIndex]) msgState = 0;
      else msgState = 4;
      //printf("\n ck0 = %X and byte = %X", ck0, ringbuffer[rdIndex]);
    }

    // Case 4:  Read & check the second checksum byte
    else if (msgState == 4) {
        msgState = 0;  // Done reading message data
        if (ck1 == ringbuffer[rdIndex])
        { // Valid message -- copy and print --- must figure out the checksum later (JK)

            //double dt_s = (rc_nanos_since_boot() - last_time) / (1e9); //calculate time since last successful reading
            //if (1 / dt_s < 5) printf("\nWARNING, Low update frequency of Serial %f (Hz)\n", 1 / dt_s); //check the update frequency
            memcpy(&serialMsg, msgdata, SERIAL_DATA_LENGTH);

            //last_time = rc_nanos_since_boot();

        }
    }
    rdIndex = RING_INC(rdIndex);
  }  
  return;
}

int send_serial_data(void)
{
    static char serial_packet[SEND_PACKET_LENGTH];
    static char data_packet[SEND_DATA_LENGTH];

    serial_packet[0] = SEND_START_BYTE0;
    serial_packet[1] = SEND_START_BYTE1;

    if (1.0/finddt_s(send_serial.time_ns) < settings.serial_send_update_hz)
    {
        send_serial_packet.flight_state = flight_status;
        send_serial_packet.time_ms = rc_nanos_since_boot() / 1000;
        //send_serial_packet.flight_state = DESCENT_TO_LAND;

        memcpy(data_packet, &send_serial_packet, SEND_DATA_LENGTH);

        fletcher16_append(data_packet, SEND_DATA_LENGTH, serial_packet + SEND_DATA_LENGTH + 2);

        memcpy(serial_packet + 2, &data_packet, SEND_DATA_LENGTH);

        if (write(serial_portID, serial_packet, SEND_PACKET_LENGTH) > 0)
        {
            /*
            printf("\nSedning  data....  fr=%f (Hz)\n", 1.0 / finddt_s(send_serial.time_ns));
            unsigned int i = 0;
            while (i < SEND_PACKET_LENGTH)
            {
                printf("\n %d Byte is: %X", i, serial_packet[i]);
                i++;
            }
            
            //printf("\nPress ENTER key to Continue\n");
            //getchar(); 
            */
            send_serial.time_ns = rc_nanos_since_boot();
        }
    }
    else
    {
        //printf("\nWaiting to send data....");
    }
    
    return 0;
}





