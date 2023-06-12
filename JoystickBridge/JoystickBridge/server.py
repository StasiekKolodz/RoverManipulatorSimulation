import socket
print("a")
# create the socket
# AF_INET == ipv4
# SOCK_STREAM == TCP
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind((socket.gethostname(), 1234))
# s.connect((socket.gethostname(), 1234))
print("b")
s.listen(5)
print("c")
while True:
    # now our endpoint knows about the OTHER endpoint.
    print("d")
    clientsocket, address = s.accept()
    print("e")
    print(f"Connection from {address} has been established.")
    clientsocket.send(bytes("Hey there!!!","utf-8"))
