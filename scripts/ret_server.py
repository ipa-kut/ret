#!/usr/bin/env python     
## Source: https://stackoverflow.com/questions/10810249/python-socket-multiple-clients                                                                                                                                                          

import socket
import thread

class RET_Server():
    def __init__(self):
        self.s = socket.socket()
        self.host = "127.0.0.1"
        self.port = 65432

        print('Server started!')
        print('Waiting for clients...')

        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(5)

        while True:
            c, addr = s.accept()     # Establish connection with client.
            print ('Got connection from', addr)
            thread.start_new_thread(on_new_client,(c,addr))
        s.close()

    def on_new_client(clientsocket,addr):
        count = 0
        while True:
            try:
                msg = clientsocket.recv(1024)
                msg = msg.decode() + " Server: " + str(count)
                print(addr, ' >> ', msg)
                count += 1
            except socket.error as socketerror:
                print (count, " Lost connection to: ", addr)  
                clientsocket.close()
                return

if __name__=="__main__":
    server = RET_Server()
