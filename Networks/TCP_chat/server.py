import socket
from threading import Thread

def sendMessage(message, clientSockets):
    for clsock in clientSockets:
        clsock.send(bytes(message))
    pass

def autorization(clsock, clientSockets, online, accounts):
    while True:
        data = clsock.recv(1000)
        if checkUser(data, accounts, online):
            print('OK')
            clsock.send(bytes("correct", 'utf8'))
            clientSockets.append(clsock)
            print("client connected from: ", addr)
            break
        else:
            clsock.send(bytes("incorrect", 'utf8'))
    pass

def reciveMessage(clsock, clientSockets, online, accounts, msg):
    autorization(clsock, clientSockets, online, accounts)
    while True:
        msg = clsock.recv(1000)
        if len(msg) == 0:
            user = clsock.recv(1000).decode()
            closeClient(clsock, clientSockets, online, user)
        else:
            if "!get" == msg.decode():
                clsock.send(bytes(str(getOnlineUsers(online)), "utf8"))
            elif "!disconnect" == msg.decode():
                usr = clsock.recv(1000).decode()
                closeClient(clsock, clientSockets, online, usr)
                break
            elif msg:
                sendMessage(msg, clientSockets)
                msg = ''
    pass

def getOnlineUsers(online):
    onUsers = []
    for key, value in online.items():
        if value:
            onUsers.append(key)
    return onUsers

def isConnected(login, online):
    if online[login]:
        return True
    return False

def closeClient(clsock, clientSockets, online, user):
    online[user] = False
    clientSockets.remove(clsock)
    clsock.close()
    pass

def checkUser(auth, acc, online):
    logAndPass = auth.decode('utf8').split('~')
    if isConnected(logAndPass[0], online):
        return False
    if logAndPass[0] in acc.keys():
        print('login ok')
        if logAndPass[1] == acc[logAndPass[0]]:
            online[logAndPass[0]] = True
            print('passwd ok')
            return True
    return False

accounts = {
    'user1': '1234',
    'user2': '5678',
    'user3': '9012'
}

online = {
    'user1': False,
    'user2': False,
    'user3': False
}

clientSockets = []
MAX_CLIENTS = 10
address = ('localhost', 8017)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(address)
sock.listen(MAX_CLIENTS)

while True:
    conn, addr = sock.accept()
    print(conn)
    threadRecv = Thread(target = reciveMessage, args = (conn, clientSockets, online, accounts, 'hello'))
    threadRecv.start()

sock.close()
