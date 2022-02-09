#!/usr/bin/env python
## Source: https://stackoverflow.com/questions/10810249/python-socket-multiple-clients                                                                                                                                                          

import socket
import thread

class RET_Server():
    def __init__(self):
        self.s = socket.socket()
        #self.host = "169.254.60.100"
        self.host = "127.0.0.1"
        self.port = 65432

        self.prbt_buffer = [None, None] # Buffer of size 2, keep last message and current msg
        self.rpi_buffer = [None, None]

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
                    count += 1
                    msg_parts = msg.split(";")
                    print(msg_parts[0], msg_parts[1], msg_parts[2])
                    if msg_parts[0] == "prbt":
                        self.prbt_buffer[0] = self.prbt_buffer[1] # Move last received message back
                        self.prbt_buffer[1] = (msg_parts[1], msg_parts[2]) # Push current message in
                        ## TODO: Write to InfluxDB
                    elif msg_parts[0] == "rpi":
                        self.rpi_buffer[0] = self.rpi_buffer[1] # Move last received message back
                        self.rpi_buffer[1] = (msg_parts[1], msg_parts[2]) # Push current message in
                        ## TODO: Write to InfluxDB
            except socket.error as socketerror:
                print (count, " Lost connection to: ", addr)  
                clientsocket.close()
                return
            except KeyboardInterrupt:
                print('Keyboard Interrupted')
                clientsocket.close()
                return

if __name__=="__main__":
    server = RET_Server()
