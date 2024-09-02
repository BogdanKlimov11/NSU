import socket
import time
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('localhost', 5556))
sock.listen(10)
srvmsg = 'World!'
while True:
    conn, addr = sock.accept()
    data = conn.recv(1024)
    if not data:
        break
    print('received message: ' + data.decode())
    #time.sleep(2)
    data = data + srvmsg.encode()
    conn.send(data)

conn.close()
