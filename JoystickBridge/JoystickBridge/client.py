import socket
import time
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.1.88", 5000))

while True:
    msg = s.recv(19)

    # print(msg.decode("utf-8"))
    # print(msg.hex())
    print(msg)
    time.sleep(1)
