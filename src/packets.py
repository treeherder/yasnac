#!/usr/bin/env python
""" Functions for encoding and decoding packets from the robot """
import struct


def encode(payload):
    """ Return a string representation of a properly encoded packet """
    length_str = struct.pack("<H2", len(payload))
    checksum = struct.pack("<H2", 65536 -
                           sum([ord(c) for c in length_str + payload]))
    return "\x02{}{}{}".format(length_str, payload, checksum)


def decode(packet):
    """
    Return an array containing decoded packet payloads. The packet argument
    can contain the data of multiple packets run together, which is useful
    because the serial buffer sometimes contains multiple packets
    """
    if not packet.startswith("\x02"):
        raise ValueError("Unknown packet format")
    length = struct.unpack("<H2", packet[1:3])[0]
    payload = packet[1:1+2+length]  # two bytes of length are included
    surplus = packet[1+2+length+2:]  # e.g. the next packet
    stated_checksum = packet[1+2+length:1+2+length+2]
    calc_checksum = struct.pack("<H2", 65536 - sum([ord(x) for x in payload]))
    if stated_checksum != calc_checksum:
        raise ValueError("Invalid packet! Checksum fail: {} != {}".format(
            stated_checksum.encode('hex'), calc_checksum.encode('hex')))
    # return a list of packets, excluding the size bytes at the beginning of
    # each payload
    return [payload[2:]] + (decode(surplus) if len(surplus) else [])
