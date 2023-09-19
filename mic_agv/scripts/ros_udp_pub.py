#!/usr/bin/env python3
import rospy #import ros library
from std_msgs.msg import String,Int16 #import message
import socket
UDP_IP = "192.168.1.255" # iot device
UDP_PORT = 1234  # iot port
def sub_udp():
    global value
    sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))
    data, addr = sock.recvfrom(1024)
    value = data.decode("utf-8", "strict")
    
def ros_pub():
    sub_udp()
    pub_1 = rospy.Publisher('udp', String, queue_size=10)
    pub_1.publish(value)
    rospy.loginfo(value)

def main():
    #Create init node
    rospy.init_node('mqtt_pub', anonymous=True) # Create init node
    rate = rospy.Rate(1) # publish rate unit hertz (1/second)
    while not rospy.is_shutdown(): # Running until ros is on
        ros_pub()
        rate.sleep() # wait until next iteration
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass