#!/usr/bin/env python3
import rospy #import ros library
from std_msgs.msg import String,Int16 #import message
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
host = '192.168.1.101'
port = 1884
topic = '/ros_mqtt'
client = mqtt.Client()
client.connect(host, port)

def sub_mqtt():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")

def ros_pub():
    sub_mqtt()
    pub_1 = rospy.Publisher('mqtt', String, queue_size=10) # define topic
    pub_1.publish(msg_payload)
    rospy.loginfo(msg_payload)

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