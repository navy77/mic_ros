#!/usr/bin/env python3
import rospy #import ros library
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
host = '192.168.1.101'
port = 1884
topic = '/ros_mqtt'
client = mqtt.Client()
client.connect(host, port)

def pub():
    global msg
    msg = "ON"
    client.publish(topic, msg)
def sub():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")
    rospy.loginfo(msg_payload)
def main():
    #Create init node
    rospy.init_node('mqtt_sub', anonymous=True) # Create init node
    rate = rospy.Rate(1) # publish rate unit hertz (1/second)
    while not rospy.is_shutdown(): # Running until ros is on
        sub() # call subscribe function 
        # pub() # call publish function
        rate.sleep() # wait until next iteration
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass