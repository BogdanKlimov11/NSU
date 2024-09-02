import time
import zmq
from pip._vendor.distlib.compat import raw_input

context = zmq.Context()
socket = context.socket(zmq.REP)
n=raw_input('server number > ')
socket.bind("tcp://*:555"+n)

while True:
    #  Wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    #  Do some 'work'
    time.sleep(0.5)

    #  Send reply back to client
    socket.send(message)