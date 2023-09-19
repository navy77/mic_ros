#!/usr/bin/env python3
import rospy #import ros library
import socket

UDP_IP = "192.168.1.4" # server
UDP_PORT = 1234  # iot port

sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)
    a = data.decode("utf-8", "strict")
    print(a)