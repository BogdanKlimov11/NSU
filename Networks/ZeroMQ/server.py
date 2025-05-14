import time
import zmq
from pip._vendor.distlib.compat import raw_input

context = zmq.Context()
socket = context.socket(zmq.REP)
n = raw_input('server number > ')
socket.bind("tcp://*:555"+n)

while True:
    # wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    # do some 'work'
    time.sleep(0.5)

    # send reply back to client
    socket.send(message)
