#!/usr/bin/env python
import socket
import sys
import time

host = "169.254.60.100" # Used for running on Pilz PC
# host = "127.0.0.1" # Used for testing on local machine
port = 65432

if __name__ == "__main__":
    print("Creating socket objects")
    conn1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Creating conn1") # Mocks the Pilz robot logs
        conn1.connect((host, port))
        print("Creating conn2") # Mocks the RPi logs
        conn2.connect((host, port))
        for i in range(1,11):
            conn1.send("prbt;"+str(time.time())+";button"+str((i%2)+1))
            time.sleep(0.5)
            conn2.send("rpi;"+str(time.time())+";button"+str((i%2)+1))
            time.sleep(0.5)
    except socket.error:
        print("A connection has failed.")
        conn1.close()
        conn2.close()
        sys.exit(0)
    conn1.close()
    conn2.close()
    print("Script done")
