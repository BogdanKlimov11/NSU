import zmq
from pip._vendor.distlib.compat import raw_input

context = zmq.Context()

# socket to talk to server
print("Connecting to server…")
socket = context.socket(zmq.REQ)
n = raw_input('enter the number of running servers > ')

for i in range(1, int(n) + 1):
    socket.connect("tcp://localhost:555" + str(i))

# do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    socket.send(str.encode('message' + str(request)))

    # get the reply.
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))
