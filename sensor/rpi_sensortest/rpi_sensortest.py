#!/usr/bin/env python

import time
import struct
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
            # Get Payload
            len = radio.getDynamicPayloadSize()
            receive_payload = bytearray(radio.read(len))
            sname, suid, ptype, spipe, batlevel = struct.unpack('=10sxBBBf', receive_payload)
            reply_type = 0
            readable = ''

            if (ptype == 1):
                readable = 'Sync'
                reply_type = 2

            elif (ptype == 3):
                readable = 'Sensor trigger'
                reply_type = 3

            elif (ptype == 4):
                readable = 'Sensor Status'
                reply_type = 3

            # Spew it
            print 'Got payload Size =', len, ' Type =', readable
            #  print "".join(map(bin,receive_payload))
            print 'Name =', sname, ' UID =', suid, ' PType =', ptype, ' SPipe =', spipe, ' BV =', batlevel

        # First, stop listening so we can talk
        radio.stopListening()

        # Build a response packet
        response = struct.pack('=10sxBBBf', sname, suid, reply_type, 0xB1, batlevel)
        radio.write(str(response))
        print 'Sent response.'
        print ''

        # Now, resume listening so we catch the next packets.
        radio.startListening()
