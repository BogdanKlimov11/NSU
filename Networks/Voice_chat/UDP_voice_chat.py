import socket
from threading import Thread
import pygame.mixer
import pyaudio
from speex import *

CHUNK = 1024
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 0.5


def sender(socket):
    FORMAT = pyaudio.paInt16    # initializing variables for recording vocie
    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    def psize(size):    # method needed for compressing message
        buff = b''
        mask = 0
        while size > 0:
            buff = bytes([(size % 2 ** 7) | mask]) + buff
            mask = 0x80
            size >>= 7
        return buff

    encoder = WBEncoder()
    print('frame size: %d' % encoder.frame_size)
    count = 0
    while True:
        vocoded = b''
        packet_size = encoder.frame_size * 2 * 1

        str_send = bytes('', 'utf-8')
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):  # loop "for" to record message
            str_send = str_send + stream.read(CHUNK)
        for i in range(0, len(str_send), packet_size):  # loop "for" to compress message
            packet = str_send[i:i + packet_size]
            if len(packet) != packet_size:
                end = len(str_send) - len(str_send) % (encoder.frame_size * 2)
                packet = str_send[i:end]

            raw = encoder.encode(packet)
            vocoded += psize(len(raw)) + raw

        count += 1
        #print(count)
        #print("message len: ", len(vocoded))
        socket.sendto(vocoded, ('255.255.255.255', 11719))


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # socket for receiving
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(('0.0.0.0', 11719))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     # socket for sending
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

pygame.mixer.init()         # initialize pygame
S = pygame.mixer.Sound      # S needed to play voice message

th_send = Thread(target=sender, args=(sock, ))
th_send.start()


def usize(buffer):      #   methods which needed to decompress message
    sizes = []
    for char in buffer:
        sizes.append(char & 0x7f)
        if char & 0x80 == 0:
            break

    size = 0
    for i in range(len(sizes)):
        size += sizes[i] * (2 ** 7) ** (len(sizes) - i - 1)
    return len(sizes), size


while True:
    message, addr = s.recvfrom(4757)
    #print("this is addr: ", addr)
    #print(addr)
    if addr[0] != '192.168.43.140':
        vocoded = message
        decoder = WBDecoder()
        i = 0
        pcm = b''
        while i < len(vocoded):     # loop "while" for decompressing message
            header_size, packet_size = usize(vocoded[i:])
            pcm += decoder.decode(vocoded[i + header_size:i + header_size + packet_size])
            i += header_size + packet_size
        S(pcm).play()       # playing message
