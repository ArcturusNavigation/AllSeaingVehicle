import socket

serverMACAddress = '70:3E:97:07:A7:B7'
port = 22
s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.connect((serverMACAddress, port))
while 1:
    s.send(bytes(0xDDA50300FFFD77))
s.close()
