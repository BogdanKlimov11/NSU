#!/usr/bin/python3
import socket
import sys
import uuid

TYPE = '0842'

mac1 = '7824AFD8902F'; 
#mac1 = 'D0DF9A22CC98';
#mac1 = 'E89D87DE0B28'; 
mac0 = '001C4240FF0F'


magic = bytes(bytearray.fromhex(''.join(['FFFFFFFFFFFF', mac1 * 16])))
packet = bytes(bytearray.fromhex(''.join([mac0, mac1, TYPE])))+magic

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
sock.bind(('enp0s5',0))
sock.send(packet)
