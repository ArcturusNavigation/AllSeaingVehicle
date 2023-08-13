import socket

host_MAC_address = "70:3E:97:07:A7:B7"
port = 3
backlog = 1
s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
size = 1024
s.bind((host_MAC_address, port))
s.listen(backlog)

try:
    client, address = s.accept()
    while True:
        data = client.recv(size)
        if data:
            print(data)
            client.send(data)
except:
    print("Closing socket")
    client.close()
    s.close()
s.close()
