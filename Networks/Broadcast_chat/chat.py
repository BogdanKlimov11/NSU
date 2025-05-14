import socket
from threading import Thread
import queue

def receiver(socket, mes):
    while True:
        message, addr = socket.recvfrom(1024)
        print('from:', addr,': ', message.decode())

def sender(socket, mes):
    while True:
        str = input()
        socket.sendto(str.encode(), ('255.255.255.255', 11719))

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(('0.0.0.0',11719))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST,1)
mes = 'hello'

th_receive = Thread(target=receiver, args = (s, mes))
th_send = Thread(target=sender, args = (sock, mes))
th_send.start()
th_receive.start()
