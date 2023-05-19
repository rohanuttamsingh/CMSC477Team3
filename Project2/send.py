import os
from socket import *

def send():
    host = '192.168.50.148' 
    port = 13000
    s = socket.socket()
    s.connect((host,port))
    message = input("->") 
    while message != 'q':
        s.send(message)
        data = s.recv(1024)
        message = input("->")
    s.close()


def send2():
    host = "192.168.50.4" # set to IP address of target computer 
    port = 13000 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 
    while True: 
        data = input("Enter message to send or type 'exit': ") 
        UDPSock.sendto(data.encode(), addr) 
        if data == "exit": 
            break 
    UDPSock.close() 
    os._exit(0) 


if __name__ == '__main__':
    # send()
    send2()


import os
from socket import *

def send():
    host = '192.168.50.148' 
    port = 13000
    s = socket.socket()
    s.connect((host,port))
    message = input("->") 
    while message != 'q':
        s.send(message)
        data = s.recv(1024)
        message = input("->")
    s.close()


def send2():
    host = "192.168.50.220" # set to IP address of target computer 
    port = 13000 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 
    while True: 
        data = input("Enter message to send or type 'exit': ") 
        UDPSock.sendto(data.encode(), addr) 
        if data == "exit": 
            break 
    UDPSock.close() 
    os._exit(0) 


if __name__ == '__main__':
    #send()
    send2()
