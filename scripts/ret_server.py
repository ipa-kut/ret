#!/usr/bin/env python
## Source: https://stackoverflow.com/questions/10810249/python-socket-multiple-clients                                                                                                                                                          

import socket
import thread
import threading
import sys
from influxdb import InfluxDBClient
from datetime import datetime
import time

class RET_Server():
    def __init__(self):
        self.robot_log = None
        self.rpi_log = None
        # Allowed delay between robot logging button press and RPI confirming it.
        # If the confirmation takes longer than this, it is to be identified as a fault.
        self.allowed_rpi_confirmation_delay = 2.0 

        self.condi_robot = threading.Condition()
        self.condi_rpi = threading.Condition()

        print("Initialising InfluxDB")
        self.client = InfluxDBClient(host="localhost",port="8086",
            username='ret', password='asdf', database="RET_Test")
        self.client.create_database("RET_Test")
        self.client.switch_database("RET_Test")

        print('Spawning Monitor')
        monitor = threading.Thread(name="monitor", target=self.monitoring_thread)
        monitor.daemon = True
        monitor.start()

        print("Starting RET Server")
        self.s = socket.socket()
        self.host = "10.4.11.132" # Used for running on Pilz PC
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

    # @brief Monitor expects that robot log always comes first. This HAS to be enforced on RPI side by
    # intentionally adding a 0.5s delay before sending socket log.
    def monitoring_thread(self):
        robot_time = 0
        robot_button = ""
        rpi_time = 0
        rpi_button = ""
        loop = 1
        while True:
            print("------------Loop {}----------------".format(loop))
            loop += 1
            try:
                # Wait for log from robot - this should always come first
                with self.condi_robot:
                    print("Waiting for log from Robot")
                    self.condi_robot.wait()
                    print("Recevied Robot log @{} - button{}"
                        .format(self.robot_log[0], self.robot_log[1]))
                    try:
                        robot_time = float(self.robot_log[0])
                        robot_button = self.robot_log[1]
                    except ValueError:
                        print("Received Robot time {} could not be cast to float"
                            .format(self.robot_log[1]))
                    print("Now wait for RPI confirmation")

                # Then wait for log from RPI - this should always come second
                with self.condi_rpi:
                    self.condi_rpi.wait()
                    print("Recevied RPI log @{} - button{}".format(self.rpi_log[0], self.rpi_log[1]))
                    try:
                        rpi_time = float(self.rpi_log[0])
                        rpi_button = self.rpi_log[1]
                    except ValueError:
                        print("Received RPI time {} could not be cast to float"
                            .format(self.rpi_log[0]))

                # Evaluate if the logs are fine
                if robot_button != rpi_button:
                    event_description = "Robot: " + robot_button + \
                        " vs RPI: " + rpi_button
                    event_type = "confirmation_mismatch"
                    print("{}: {}".format(event_type, event_description))
                    self.write_event_to_influxdb(event_type, event_description)
                    print("")
                elif abs(robot_time - rpi_time) > self.allowed_rpi_confirmation_delay:
                    event_description = "Actual: " + repr(abs(robot_time - rpi_time)) + \
                        "s vs Allowed: " \
                             + repr(self.allowed_rpi_confirmation_delay) + "s."
                    event_type = "confimration_timeout"
                    print("{}: {}".format(event_type, event_description))
                    self.write_event_to_influxdb(event_type, event_description)
                    print("")
                else:
                    print("RPI Succesfully confirmed Robot")
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
                    # print(msg_parts[0], msg_parts[1], msg_parts[2])
                    # msg parts are:
                    # 0 -> source of log. Options: "prbt", "rpi", "ur_native" and "ur_ros"
                    # 1 -> epoch time. Ex: "1234567.4567"
                    # 2 -> button number that was pressed. Ex: "1" or "2"
                    if msg_parts[0] == "prbt" or msg_parts[0] == "ur_native"  or msg_parts[0] == "ur_ros":
                        with self.condi_prbt:
                            self.prbt_log = (msg_parts[1], msg_parts[2])
                            self.condi_prbt.notifyAll()
                    elif msg_parts[0] == "rpi":
                        with self.condi_rpi:
                            self.rpi_log = (msg_parts[1], msg_parts[2])
                            self.condi_rpi.notifyAll()
                    else: 
                        print("Could not manage the following input data")
                        print(msg)
                        
                    self.write_log_to_influxdb(msg_parts)
            except socket.error as socketerror:
                print (count, " Lost connection to: ", addr)  
                clientsocket.close()
                return
            except KeyboardInterrupt:
                print('Socket server Keyboard Interrupted')
                clientsocket.close()
                return

    def write_log_to_influxdb(self, log):
        json = [{
            
            "measurement": "RET_Logs_"+datetime.today().strftime('%Y-%m-%d'),
            "tags": {
                "source": log[0]
            },
            
            "time": datetime.utcfromtimestamp(float(log[1])).strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
            "fields": {
                "datetime": datetime.utcfromtimestamp(float(log[1])).strftime('%Y-%m-%d %H:%M:%S.%fZ')[:-5],
                "button": int(log[2]),
            }
        }]
        self.client.write_points(json)

    def write_event_to_influxdb(self, event_type, event_description):
        json = [{
            "measurement": "RET_Events_"+datetime.today().strftime('%Y-%m-%d'),
            "tags": {
                "type": event_type,
            },
            "time": datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
            "fields": {
                "description": event_description,
            }
        }]
        self.client.write_points(json)

if __name__=="__main__":
    server = RET_Server()
