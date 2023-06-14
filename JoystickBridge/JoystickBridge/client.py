import socket
import time
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.1.88", 5000))
    mystr = ""
    s.recv(11)
    # data = 12
    mystrk = []
    for i in range(19):
        mystr = 'k'
        mystrk.append(i)
        # print(len(k))
        # data = bytes(mystr, 'utf-8')
    data = bytes(mystrk)
    print(len(data))
    s.sendall(data)
    print("succes")
    buffer = []
    for i in range(19):
        ret = s.recv(1)
        buffer.append(ret.hex())
    
    print(f"Ret: {buffer}")
    # time.sleep(3)
        

#         string = "Python is interesting."

# # string with encoding 'utf-8'
# arr = bytes(string, 'utf-8')
except Exception as e:
    print(e)
# while True:
#     msg = s.recv(19)

#     # print(msg.decode("utf-8"))
#     # print(msg.hex())
#     print(msg)
#     time.sleep(1)
