#!/usr/bin/env python     
## Source: https://stackoverflow.com/questions/10810249/python-socket-multiple-clients                                                                                                                                                          

import socket
import thread

class RET_Server():
    def __init__(self):
        self.s = socket.socket()
        self.host = "169.254.60.100"
        self.port = 65432

        print('Server started!')
        print('Waiting for clients...')

        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.host, self.port))
        self.s.listen(5)

        while True:
            c, addr = self.s.accept()
            print ('Got connection from', addr)
            thread.start_new_thread(self.on_new_client,(c,addr))
        self.s.close()

    def on_new_client(self, clientsocket,addr):
        count = 0
        while True:
            try:
                msg = clientsocket.recv(1024)
                if msg:
                    print(addr, ' >> ', count, ":", msg)
                    count += 1
            except socket.error as socketerror:
                print (count, " Lost connection to: ", addr)  
                clientsocket.close()
                return

if __name__=="__main__":
    server = RET_Server()
