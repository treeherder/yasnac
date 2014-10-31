    #!/usr/bin/env python
""" Library for communicating with a YASNAC ERC-series robot """
import struct
import sys
import time
import collections
import os

import serial


DEBUG = True

# The erc serial protocol is *very* similar to:
# https://en.wikipedia.org/wiki/Binary_Synchronous_Communications

SOH = chr(0x01)  # Start Of Heading: denotes the start of the message heading
                 # data block

STX = chr(0x02)  # Start Of Text: denotes end of heading and beginning of
                 # information data

ETX = chr(0x03)  # End Of Text: signals receiving station that all payload
                 # data has been transmitted

EOT = chr(0x04)  # End Of Transmission: indicates the end of transmission of
                 # all data associated with a message.

ENQ = chr(0x05)  # Enquiry: requests a response from the receiving station.

DLE = chr(0x10)  # Data Link Escape: is used to modify the meaning of a
                 # limited number of subsequent characters.

NAK = chr(0x15)  # Negative Acknowledge: indicates improper communication.

ETB = chr(0x17)  # End of Transmission Block: indicates the end of a
                 # particular block of transmitted data. ETB is used in place
                 # of ETX when data are transmitted in two or more blocks.

ACK0 = DLE + chr(0x30)  # Even acknowledgment

ACK1 = DLE + chr(0x31)  # Odd acknowledgment

WACK = DLE + chr(0x6b)  # Wait acknowledgement

RVI = DLE + chr(0x7c)  # Reverse interrupt

TTD = STX + ENQ  # Temporary transmission delay

CONTROL_CHARS = {
    SOH: "SOH",
    STX: "STX",
    ETX: "ETX",
    EOT: "EOT",
    ENQ: "ENQ",
    DLE: "DLE",
    NAK: "NAK",
    ETB: "ETB",
    ACK0: "ACK0",
    ACK1: "ACK1",
    WACK: "WACK",
    RVI: "RVI",
    TTD: "TTD"
}

TRANSACTIONS = {
    '01,000': 'command from remote computer',

    # job and special system files - transmission
    '02,001': '"JOBNAME".JBI - indicates independent job data',
    '02,002': '"JOBBAME".JBR - indicates related (master) job data',
    '02,011': 'WEAV.DAT - weave data',
    '02,012': 'TOOL.DAT - tool data',
    '02,013': 'UFRAME.DAT - user coordinate data',
    '02,014': 'ABSWELD.DAT - welder condition data',  # undocumented
    '02,015': 'CV.DAT - conveyer condition data',
    '02,016': 'SENSOR.DAT - locus correction condition data',
    '02,017': 'COMARC2.DAT - com-arc 2 condition data',
    '02,018': 'PC1PC2.DAT - phase comprehension data',
    '02,020': 'POSOUT.DAT - unknown',  # undocumented
    '02,022': 'RECIPRO.DAT - unknown',  # undocumented
    '02,023': 'PALACT.DAT - palletizing action data',  # undocumented
    '02,030': 'SYSTEM.DAT - system data',

    # job and special system files - request
    '02,051': '"JOBNAME".JBI - request for independent job data',
    '02,052': '"JOBNAME".JBR - request for related (master) job data',
    '02,061': 'WEAV.DAT - request for weave data',
    '02,062': 'TOOL.DAT - request for tool data',
    '02,063': 'UFRAME.DAT - request for user coordinate data',
    '02,064': 'ABSWELD.DAT - request for welder condition data',  # undocumented
    '02,065': 'CV.DAT - request for conveyer condition',
    '02,066': 'SENSOR.DAT - request for locus correction condition data',
    '02,067': 'COMARC2.DAT - request for COM-ARC2 condition data',
    '02,068': 'PC1PC2.DAT - request for phase comprehension data',
    '02,070': 'POSOUT.DAT - request for unknown',  # undocumented
    '02,072': 'RECIPRO.DAT - request for unknown',  # undocumented
    '02,073': 'PALACT.DAT - request for palletizing action data',  # undoc'd.
    '02,080': 'SYSTEM.DAT - request for system data',

    # variables data - transmission
    '03,001': 'byte variable',
    '03,002': 'integer variable',
    '03,003': 'double precision variable',
    '03,004': 'real variable',
    '03,005': 'position variable (pulse data)',
    '03,006': 'position variable (rectangular data)',
    '03,007': 'external axis variable (pulse data)',
    '03,008': 'external axis variable (rectangular data)',

    # variable data - request
    '03,051': 'request for byte variable',
    '03,052': 'request for integer variable',
    '03,053': 'request for double precision variable',
    '03,054': 'request for real variable',
    '03,055': 'request for position variable (pulse data)',
    '03,056': 'request for position variable (rectangular data)',
    '03,057': 'request for external axis variable (pulse data)',
    '03,058': 'request for external axis variable (rectangular data)',

    # job execution response
    '90,000': '0000 or a 4 digit error code, response to a command',
    '90,001': 'data response, variable number of digits/data sent as csv'
}

ERRORS = {
    # 1xxx - command test
    '1010': 'command failure',
    '1011': 'command operand number failure',
    '1012': 'command operand value excessive',
    '1013': 'command operand length failure',
    
    # 2xxx - command execution mode error
    '2010': 'during robot operation',
    '2020': 'during T-PENDANT',
    '2030': 'during panel HOLD',
    '2040': 'during external HOLD',
    '2050': 'during command HOLD',
    '2060': 'during error alarm',
    '2070': 'in servo OFF or stopping by a panel HOLD',
    
    # 3xxx - command execution error
    '3010': 'servo power on',
    '3040': 'set home position',
    '3070': 'current position is not input',
    '3080': 'END command of job (except master job)',
    
    # 4xxx - job registration error
    '4010': 'shortage of memory capacity (job registration)',
    '4012': 'shortage of memory capacity (position data registration)',
    '4020': 'job edit prohibit',
    '4030': 'job of same name exists',
    '4040': 'no desired job',
    '4060': 'set execution',
    '4120': 'position data broken',
    '4130': 'no position data',
    '4150': 'END command of job (except master job)',
    '4170': 'instruction data broken',
    '4190': 'unsuitable characters in job name exist',
    '4200': 'unsuitable characters in job name exist',
    '4230': 'instructions which cannot be used by this system exist',
    
    # 5xxx - file text error
    '5110': 'instruction syntax error',
    '5120': 'position data fault',
    '5130': 'neither NOP or END exists',
    '5170': 'format error',
    '5180': 'data number is inadequate',
    '5200': 'data range exceeded'
}


Packet = collections.namedtuple("Packet", 'body header footer packet_length')


class InvalidPacketStart(Exception):
    pass


class InvalidPacketNeedMore(Exception):
    pass


class InvalidPacketBody(Exception):
    pass


class InvalidPacketChecksum(Exception):
    pass


class InvalidTransaction(Exception):
    pass


def log(message):
    """ Print the given message to stdout, flush stdout """
    sys.stdout.write(message + "\n")
    sys.stdout.flush()
    return message


def warn(message, force=False):
    """ Print the given message to stderr, flush stderr """
    if DEBUG or force:
        sys.stderr.write(message + "\n")
        sys.stderr.flush()
    return message


def chunks(iterable, chunksize):
    """
    Yield successive n-sized chunks from the given iterable
    From http://stackoverflow.com/questions/312443
    """
    for i in xrange(0, len(iterable), chunksize):
        yield iterable[i:i+chunksize]

def multifind(haystack, needles, start=None, end=None):
    """
    like string.find, except the search term is a list of searches that will
    be tried in sequence. the index of the first found match will be returned.
    if no match is found, -1 will be returned
    """
    for index, char in enumerate(haystack):
        if char in needles:
            return index
    return -1

def checksum(packet, stop, start=1):
    """ Return the checksum for the given packet """
    return sum([ord(c) for c in packet[start:stop]])

def encode(transaction_code, body):
    # return a list of packets to send, based on the given code and body
    body_chunks = list(chunks(body, 256))
    chunk_count = len(body_chunks)
    result = list()

    # handle the first packet
    packet = "".join([SOH, transaction_code, STX, body_chunks.pop(0),
                      ETB if chunk_count > 1 else ETX])
    packet += struct.pack("<H", checksum(packet, len(packet) + 2))
    result.append(packet)
    
    # ...and now the rest
    for index, chunk in enumerate(body_chunks):
        packet = "".join([STX, chunk, ETB if index < chunk_count else ETX])
        packet += struct.pack("<H", checksum(packet, len(packet) + 2))
        result.append(packet)

    return result

def decode(packet):
    """ read packet data from the input buffer and return decoded data """
    if packet.startswith(SOH):
        header_bytes = 8
        header = struct.unpack("c6sc", packet[0:header_bytes])[1]
    elif packet.startswith(STX):
        header_bytes = 1
        header = struct.unpack("c", packet[header_bytes])
    elif packet in CONTROL_CHARS:
        raise InvalidPacketStart(("Attempt to decode {} control char as "
                                  "packet").format(CONTROL_CHARS[packet]))
    else:
        raise InvalidPacketStart("{}".format(packet.__repr__()))

    max_search_length = 256 + header_bytes + 1
    body_end = multifind(packet, (ETX, ETB), start=header_bytes,
                         end=max_search_length)
    if body_end < 0:
        if len(packet) < max_search_length:
            raise NeedMoreData
        else:
            raise InvalidPacketBody

    body = packet[header_bytes:body_end]
    packet_length = body_end + 3
    # For your use while debugging:
    # print "body {}".format(body.__repr__())
    # print "body_end {}".format(body_end)
    # print "packet_length {}".format(packet_length)
    # print "included body {}".format(packet[body_end:packet_length].__repr__())
    # print "checksum body {}".format(packet[1:packet_length].__repr__())
    footer = struct.unpack("<cH", packet[body_end:packet_length])
    calculated_checksum = checksum(packet, packet_length - 2)
    if footer[1] != calculated_checksum:
        raise InvalidPacketChecksum("{} != {}".format(footer[1],
                                                      calculated_checksum))
    return Packet(body, header, footer, packet_length)


def test():
    print "entering erc comms loop"
    x=ERC()
    x.loop()


class ERC(object):
    handlers = None
    link = None
    ack_bit = False

    def __init__(self):
        self.handlers = dict({
            '02,001': self.handle_incoming_file,
            '02,002': self.handle_incoming_file,
            '02,011': self.handle_incoming_file,
            '02,012': self.handle_incoming_file,
            '02,013': self.handle_incoming_file,
            '02,014': self.handle_incoming_file,
            '02,015': self.handle_incoming_file,
            '02,016': self.handle_incoming_file,
            '02,017': self.handle_incoming_file,
            '02,018': self.handle_incoming_file,
            '02,020': self.handle_incoming_file,
            '02,022': self.handle_incoming_file,
            '02,023': self.handle_incoming_file,
            '02,030': self.handle_incoming_file,
            '02,051': self.handle_file_request,
        })
        self.link = serial.Serial(port='/dev/ttyS0',
                                  baudrate=9600,
                                  bytesize=8,
                                  parity=serial.PARITY_EVEN, 
                                  stopbits=serial.STOPBITS_ONE,
                                  timeout=None)
        self.ack_bit = False

    def raw_read(self):
        """ Return the contents of incoming raw data on the serial port """
        input_buffer = []
        while not self.link.inWaiting():
            time.sleep(0.05)
        while self.link.inWaiting():
            input_buffer.append(self.link.read(size=self.link.inWaiting()))
            time.sleep(0.015)
        result = "".join(input_buffer)
        warn("raw_read {} bytes: {}".format(len(result), result.__repr__()))
        return result

    def raw_write(self, message):
        """ Send raw data on the serial port """
        self.link.write(message)
        warn("raw_write {} bytes: {}".format(len(message),
                                             message.__repr__()))
    
    def send_ack(self):
        """
        Send the appropriate ACK message, which alternates between ACK0 and ACK1
        """
        self.raw_write(self.return_ack())

    def return_ack(self):
        """ Return the appropriate ACK message to the calling function """
        result = ACK1 if self.ack_bit else ACK0
        self.ack_bit = not self.ack_bit
        return result

    def handle_eot(self, write=False, read=False):
        """ handle an EOT, which resets the ACK bit """
        if read:
            # There should be an EOT on the wire. Drain it.
            raw_packet = self.raw_read()
            if raw_packet != EOT:
                raise InvalidTransaction("Expected EOT, got {}".format(
                    raw_packet.__repr__()))

        if write:
            self.raw_write(EOT)

        self.ack_bit = False


    def handshake(self, active=True):
        """ Ping the robot """
        if active:
            # send ENQ then listen for an ACK0/ACK1
            self.raw_write(ENQ)
            
        raw_packet = self.raw_read()
        
        # if we sent an ENQ, we expect ACK0/ACK1. otherwise we should see ENQ
        expected_input = self.return_ack() if active else ENQ
        
        if raw_packet != expected_input:
            raise InvalidTransaction("Expecting {}, got {}".format(
                expected_input.__repr__(), raw_packet.__repr__()))
        
        if not active:
            # We heard an ENQ, now we send the ack
            self.send_ack()
        
        return True

    def read_multipacket_message(self):
        """ read ETB packets until an ETX package is received """
        result = ""
        while True:
            raw_packet = self.raw_read()
            packet = decode(raw_packet)
            print "DRAINING MULTIPACKET {}".format(packet.header)
            result += packet.body
            self.send_ack()
            if packet.footer[0] == ETX:
                break
        self.handle_eot(read=True)
        print "DRAIN COMPLETE"
        return result

    def confirmed_write(self, packet):
        """ Send the given packet, wait for the appropriate ack """
        confirmed = False
        while not confirmed:
            self.raw_write(packet)
            expected_ack = self.return_ack()
            raw_packet = self.raw_read()
            if raw_packet == expected_ack:
                confirmed = True
            else:
                print "wrong ack, got {} expected {}".format(
                    raw_packet.__repr__(), expected_ack.__repr__())
        return confirmed

    def handle_incoming_file(self, header, message):
        """
        Handle incoming file transfers;
        - save the file to disk
        - send a properly formatted reply message to the yasnac
        """
        payload = message.splitlines()
        
        if header == '02,001':
            filename_extension = "JBI"  # independent job data
        elif header == '02,002':
            filename_extension = "JBR"  # related (master) job data
        else:
            filename_extension = "DAT"
        
        filename = "{}.{}".format(payload.pop(0), filename_extension)
        
        # Write the file
        # fixme: safety-check the filename
        with open(filename, "w") as fileout:
            fileout.write("\r".join(payload) + "\r")
        
        # The machine should have hung up already
        self.handshake()
        
        # Now we send back the higher-level transfer confirmation...
        self.confirmed_write(encode("90,000", "0000\r")[0])
        
        self.handle_eot(write=True)
        
        return (filename, "\r".join(payload))

    def handle_file_request(self, header, message):
        """
        Handle a request for a file.
        - load the file from disk
        - send the proper replies
        """        
        if header == '02,051':
            filename_extension = "JBI"  # independent job data
        elif header == '02,052':
            filename_extension = "JBR"  # related (master) job data
        else:
            filename_extension = "DAT"

        filename = "{}.{}".format(message.strip(), filename_extension)
        # fixme: safety-check the filename
        if not os.path.exists(filename):
            pass
        # fixme: finish this!
        
    def system_control_command(self, command_string):
        """ Issue a system control command """
        self.handshake()
        self.confirmed_write(encode("01,000", command_string)[0])
        self.handle_eot(write=True)
        self.handshake(active=False)
        raw_packet = self.raw_read()
        packet = decode(raw_packet)
        if packet.header != "90,000" or packet.body != '0000\r':
            warn("expected 90,000 with 0000, got {}".format(packet))
        self.send_ack()
        self.handle_eot(read=True)

    def servos_on(self):
        """ Shorthand for servo_power(True) """
        return self.servo_power(True)

    def servos_off(self):
        """ Shorthand for servo_power(False) """
        return self.servo_power(False)

    def servo_power(self, power=True):
        """ EXPERIMENTAL: tell the ERC to turn on/off the servos """
        self.system_control_command("SVON {}\r".format("1" if power else "0"))

    def start(self, job=None):
        """ EXPERIMENTAL: tell the ERC to run a job """
        self.system_control_command("START {}\r".format(job if job else ""))

    def hold(self, hold=True):
        """ EXPERIMENTAL: tell the ERC to stop or clear the stop flag """
        self.system_control_command("HOLD {}\r".format("1" if hold else "0"))

    def loop(self):
        while True:
            raw_packet = self.raw_read()

            # raw packet handlers
            if raw_packet == ENQ:
                self.send_ack()
                continue

            if raw_packet == EOT:
                print "received EOT"
                self.handle_eot()
                continue
            
            if not raw_packet.startswith(SOH):
                print "No handler for packet {}".format(raw_packet.__repr__())
                continue

            # parsed packet handlers (packet begins with SOH)
            packet = decode(raw_packet)

            print "packet header: {}".format(packet.header)

            message = packet.body
            self.send_ack()

            # is this a multi-packet message? if so, read the whole message
            if packet.footer[0] == ETB:
                # keep capturing packets until an ETX
                message += self.read_multipacket_message()

            if packet.header in self.handlers:
                print "handled packet(s), result was: {}".format(
                    self.handlers[packet.header](packet.header, message))
                continue
            else:
                print "Don't know how to handle code {}".format(packet.header)

            print "received {} message: {}".format(TRANSACTIONS[packet.header],
                                                   message.__repr__())
            
