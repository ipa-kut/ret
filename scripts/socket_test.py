from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import rospy
import socket, sys
import time



__SOCKET_HOST__ = '169.254.60.100'
__SOCKET_PORT__ = 65432

if __name__ == "__main__":

    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        connection.connect((__SOCKET_HOST__, __SOCKET_PORT__))
        print("connection started")
        time.sleep(1)
        connection.sendall("asdf")
        time.sleep(1)
        connection.sendall("asdf")
        time.sleep(1)
        connection.sendall("asdf")
        time.sleep(1)
        connection.sendall("asdf")
        
    except socket.error:
        print("Connection has failed.")
        connection.close()
        sys.exit(0)
    print("Connection established with the server.")
