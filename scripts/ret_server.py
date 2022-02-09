#!/usr/bin/env python
## Source: https://stackoverflow.com/questions/10810249/python-socket-multiple-clients                                                                                                                                                          

import socket
import thread
import threading
import sys

class RET_Server():
    def __init__(self):
        self.prbt_log = None
        self.rpi_log = None
        # Allowed delay between PRBT logging button press and RPI confirming it.
        # If the confirmation takes longer than this, it is to be identified as a fault.
        self.allowed_rpi_confirmation_delay = 2.0 

        self.condi_prbt = threading.Condition()
        self.condi_rpi = threading.Condition()

        print('Spawning Monitor')
        monitor = threading.Thread(name="monitor", target=self.monitoring_thread)
        monitor.daemon = True
        monitor.start()

        print("Starting RET Server")
        self.s = socket.socket()
        self.host = "169.254.60.100" # Used for running on Pilz PC
        # self.host = "127.0.0.1" # Used for testing on local machine
        self.port = 65432
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.host, self.port))
        self.s.listen(5)
        print('Server started, waiting for clients...')

        while True:
            try:
                c, addr = self.s.accept()
                print ('Got connection from', addr)
                thread.start_new_thread(self.on_new_client,(c,addr))
            except KeyboardInterrupt:
                print('Main program Keyboard Interrupted')
                self.s.close()
                sys.exit()

    # @brief Monitor expects that PRBT log always comes first. This HAS to be enforced on RPI side by
    # intentionally adding a 0.5s delay before sending socket log.
    def monitoring_thread(self):
        prbt_time = 0
        prbt_button = ""
        rpi_time = 0
        rpi_button = ""
        loop = 1
        while True:
            print("------------Loop {}----------------".format(loop))
            loop += 1
            try:
                # Wait for log from PRBT - this should always come first
                with self.condi_prbt:
                    print("Waiting for log from PRBT")
                    self.condi_prbt.wait()
                    print("Recevied PRBT log: {} - {}"
                        .format(self.prbt_log[0], self.prbt_log[1]))
                    try:
                        prbt_time = float(self.prbt_log[0])
                        prbt_button = self.prbt_log[1]
                    except ValueError:
                        print("Received PRBT time {} could not be cast to float"
                            .format(self.prbt_log[1]))
                    print("Now wait for RPI confirmation")

                # Then wait for log from RPI - this should always come second
                with self.condi_rpi:
                    self.condi_rpi.wait()
                    print("Recevied RPI log: {} - {}".format(self.rpi_log[0], self.rpi_log[1]))
                    try:
                        rpi_time = float(self.rpi_log[0])
                        rpi_button = self.rpi_log[1]
                    except ValueError:
                        print("Received RPI time {} could not be cast to float"
                            .format(self.rpi_log[0]))

                # Evaluate if the logs are fine
                if prbt_button != rpi_button:
                    print("PRBT reported {} press but RPI reported {} press. Fault!!"
                        .format(prbt_button, rpi_button))
                    print("")
                    ## TODO: Write to InfluxDB
                elif abs(prbt_time - rpi_time) > self.allowed_rpi_confirmation_delay:
                    print("PRBT - RPI Confirmation took {}, \
                        which longer than the allowed delay {}. Fault!!"
                        .format(abs(prbt_time - rpi_time), self.allowed_rpi_confirmation_delay))
                    print("")
                    ## TODO: Write to InfluxDB
                else:
                    print("RPI Succesfully confirmed PRBT")
                    print("")

            except KeyboardInterrupt:
                print('Monitor Keyboard Interrupted')
                return

    def on_new_client(self, clientsocket, addr):
        count = 0
        while True:
            try:
                msg = clientsocket.recv(1024)
                if msg:
                    count += 1
                    msg_parts = msg.split(";")
                    print(msg_parts[0], msg_parts[1], msg_parts[2])
                    if msg_parts[0] == "prbt":
                        with self.condi_prbt:
                            self.prbt_log = (msg_parts[1], msg_parts[2])
                            ## TODO: Write to InfluxDB
                            self.condi_prbt.notifyAll()
                    elif msg_parts[0] == "rpi":
                        with self.condi_rpi:
                            self.rpi_log = (msg_parts[1], msg_parts[2])
                            ## TODO: Write to InfluxDB
                            self.condi_rpi.notifyAll()
            except socket.error as socketerror:
                print (count, " Lost connection to: ", addr)  
                clientsocket.close()
                return
            except KeyboardInterrupt:
                print('Socket server Keyboard Interrupted')
                clientsocket.close()
                return

if __name__=="__main__":
    server = RET_Server()
