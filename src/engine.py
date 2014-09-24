#!/usr/bin/env python
""" Emulate the robodisk """
from time import sleep
import sys
import os

import serial

import packets


def log(message):
    """ Print the given message to stdout, flush stdout """
    sys.stdout.write(message + "\n")
    sys.stdout.flush()


def warn(message):
    """ Print the given message to stderr, flush stderr """
    sys.stderr.write(message + "\n")
    sys.stderr.flush()


def chunks(iterable, chunksize):
    """
    Yield successive n-sized chunks from the given iterable
    From http://stackoverflow.com/questions/312443
    """
    for i in xrange(0, len(iterable), chunksize):
        yield iterable[i:i+chunksize]


class YASNAC(object):
    """ Emulate the YASNAC disk controller """
    com = None

    def __init__(self):
        self.com = serial.Serial(port='/dev/ttyS0', baudrate=4800,
                                 parity=serial.PARITY_EVEN, timeout=0)
        sleep(1)  # wait for the port to be ready (an arbitrary period)
        log("opened serial port")

    def raw_read(self):
        """ Return the contents of incoming raw data on the serial port """
        return self.com.read(size=self.com.inWaiting())

    def raw_write(self, message):
        """ Send a raw packet on the serial port """
        self.com.write(message)

    def read(self):
        """ return decoded packet(s) from the serial port """
        return packets.decode(self.raw_read())

    def write(self, message):
        """ encode and send the given message to the serial port """
        self.raw_write(packets.encode(message))
        sleep(0.25)

    def confirmed_write(self, message, limit=10):
        """ send a message, repeating as needed until we get an ack """
        message_received = False
        while not message_received:
            self.write(message)
            if self.read()[-1] == 'ACK':
                message_received = True
            else:
                limit -= 1
            if limit < 1:
                raise RuntimeError(("Unable to get confirmation on write "
                                    "of {}").format(message.encode('hex')))
        return True

    def handshake(self):
        """ Connect to the yasnac """
        if self.read()[-1] == "ENQ":
            self.write('ACK')
            return True
        raise EnvironmentError("handshake failed, no inquiry heard")

    def emulate_loop(self):
        """ Loop, responding to the device as needed """
        self.handshake()

        while True:
            packet = self.read()[-1]

            if packet == 'ENQ':
                self.write('ACK')
                continue
            if packet == 'ACK':
                self.write('EOF')
                continue
            if packet == 'LST':
                for filename in os.listdir('.'):
                    if not filename.endswith('.JBI') or len(filename) > 16:
                        continue
                    self.confirmed_write("LST{:16}".format(filename))
                self.write("EOF")
            if packet == 'DSZ':
                self.write("DSZ00729088")
                continue
            if packet.startswith("FRD"):
                filename = packet[3:].rstrip()
                with open(filename) as inputfh:
                    filedata = inputfh.read()
                # send the file in 255 character blocks, retrying as necessary
                for chunk in chunks(filedata, 255):
                    self.confirmed_write(chunk)
                self.write("EOF")


def main():
    """ Primary function for direct execution """
    moto = YASNAC()
    moto.emulate_loop()


if __name__ == '__main__':
    main()
