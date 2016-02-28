#!/usr/bin/env python

import time
from RF24 import *

radio = RF24(22, RPI_BPLUS_GPIO_J8_24, BCM2835_SPI_SPEED_8MHZ)

control_pipes = [0xA0A0A0A0E1, 0xF0F0F0F0A1]
data_pipe = 0xF0F0F0F0B1

print 'Sensor Test'
radio.begin()
radio.enableDynamicPayloads()
radio.setRetries(5,15)
radio.setPALevel(RF24_PA_LOW)
radio.printDetails()

print '**************************************************'
radio.openWritingPipe(control_pipes[0])
radio.openReadingPipe(1,control_pipes[1])
radio.openReadingPipe(2,data_pipe)
radio.startListening()

reply_type = 0

# forever loop
while 1:
    if radio.available():
        while radio.available():
            # Fetch the payload, and see if this was the last one.
            len = radio.getDynamicPayloadSize()

            receive_payload = bytearray(radio.read(len))
            packet_type = receive_payload[12]
            reply_type = 0

            readable = ''
            if (packet_type == 4):
                # Sync Packet, send back 2 LSB of data pipe
                readable = 'Sync'
                receive_payload.append(0xB1)
                reply_type = 2
                #  receive_payload[12] = 0x2

            elif (packet_type == 1):
                readable = 'Sensor trigger'
            elif (packet_type == 3):
                readable = 'Sensor Status'

            # Spew it
            print 'Got payload size=', len, ' type=', readable
            #  for c in receive_payload:
                #  print bin(ord(c))
            print receive_payload
            print "".join(map(bin,receive_payload))
            print ''

        # First, stop listening so we can talk
        radio.stopListening()

        # Send the final one back.
        if reply_type > 0:
            receive_payload[12] = reply_type

        print receive_payload
        print "".join(map(bin,receive_payload))
        print ''
        radio.write(str(receive_payload))
        print 'Sent response.'

        # Now, resume listening so we catch the next packets.
        radio.startListening()
