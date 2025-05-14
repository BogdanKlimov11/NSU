import socket
import time

msg = 'Hello!'
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 9091))
sock.send(msg.encode())
time.sleep(2)
msg = sock.recv(1024)
print('message from server: ' + msg.decode())
sock.close()
