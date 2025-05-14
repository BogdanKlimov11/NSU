#!/usr/bin/python3

import socket
import sys
import uuid

TYPE = '0842'

mac1 = '7824AFD8902F'
#mac1 = 'D0DF9A22CC98';
#mac1 = 'E89D87DE0B28';
mac0 = 'B88D12560DBC'

magic = bytes(bytearray.fromhex(''.join(['FFFFFFFFFFFF', mac1 * 16])))
packet = bytes(bytearray.fromhex(''.join([mac1, mac0, TYPE]))) + magic

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
sock.bind(('eno1',0))
sock.send(packet)
