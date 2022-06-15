import socket
from alexnet import position
import pickle

HOST = ''
PORT = 50007
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)


def pos(cx):
    if cx[0]:
        cx = 200  # no line detected

    elif cx[1]:
        cx = 0

    elif cx[2]:
        cx = 22

    elif cx[3]:
        cx = 37

    elif cx[4]:
        cx = 52

    elif cx[5]:
        cx = 67

    elif cx[6]:
        cx = 80

    elif cx[7]:
        cx = 92

    elif cx[8]:
        cx = 107

    elif cx[9]:
        cx = 122

    elif cx[10]:
        cx = 137

    else:
        cx = 160

    return cx


while 1:
    data = conn.recv(4096)
    if data:
        if data == [1]:
            break
        data = pickle.loads(data)
        data = position(data)
        data = pos(data)
        data = pickle.dumps(data)
        conn.send(data)

conn.close()
