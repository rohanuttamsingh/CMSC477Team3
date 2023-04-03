import os
from socket import *

def get():
    host = '192.168.50.148'
    port = 13000
    s = socket.socket()
    s.bind((host, port))
    s.listen(1)
    c, addr = s.accept()
    while True:
        data = c.recv(1024)
        if not data:
            break
        data = str(data).upper()
        c.send(data)
    c.close()

def get2():
   host = "" 
   port = 13000 
   buf = 1024 
   addr = (host, port) 
   UDPSock = socket(AF_INET, SOCK_DGRAM) 
   UDPSock.bind(addr) 
   print ("Waiting to receive messages...")
   while True: 
      (data, addr) = UDPSock.recvfrom(buf) 
      print ("Received message: " + data )
      if data == "exit": 
         break 
   UDPSock.close() 
   os._exit(0) 

if __name__ == '__main__':
    get()
    #get2()



