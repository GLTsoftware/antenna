#!/usr/bin/env python3
import socket
import sys
import struct
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_address=('88.83.29.82',7000)
sock.connect(server_address)
sock.send(bytearray([0xad,0x0,0x0,0x0,0x0]))
data=sock.recv(1024)
print(data)
