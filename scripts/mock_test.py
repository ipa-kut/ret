#!/usr/bin/env python
import socket
import sys
import time

host ="10.4.11.132" # Used for running on Pilz PC
#host = "127.0.0.1" # Used for testing on local machine
port = 65432

if __name__ == "__main__":
    robot = "mock_robot" if sys.argv[1] == "" else sys.argv[1]
    print("Creating socket objects")
    conn1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Creating conn1") # Mocks the Pilz robot logs
        conn1.connect((host, port))
        print("Creating conn2") # Mocks the RPi logs
        conn2.connect((host, port))
        for i in range(1,6):
            conn1.send(robot+";"+repr(time.time())+";"+str((i%2)+1))
            time.sleep(0.5)
            conn2.send("rpi;"+repr(time.time())+";"+str((i%2)+1))
            time.sleep(0.5)
        # Now simulate confirmation mismatch
        conn1.send(robot+";"+repr(time.time())+";"+str((i%2)+1))
        time.sleep(0.5)
        conn2.send("rpi;"+repr(time.time())+";"+str(2-(i%2)))
        time.sleep(0.5)
        # Now simulate confirmation timeout
        conn1.send(robot+";"+repr(time.time())+";"+str((i%2)+1))
        time.sleep(3) # Should be greater than self.allowed_rpi_confirmation_delay in ret_server.py
        conn2.send("rpi;"+repr(time.time())+";"+str((i%2)+1))
        time.sleep(0.5)
    except socket.error:
        print("A connection has failed.")
        conn1.close()
        conn2.close()
        sys.exit(0)
    conn1.close()
    conn2.close()
    print("Script done")
