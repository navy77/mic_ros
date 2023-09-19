#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float64,Float32MultiArray
from geometry_msgs.msg import Twist
import time

def callback1(Twist):
    rospy.loginfo(Twist.linear.x)

def callback2(data):
    # rospy.loginfo("%s"+"::"+"%s" ,data.data[0],data.data[1])
    a = data.data[0]
    b = data.data[1]
    rospy.loginfo("%s",a+b)

def sub_cmd():
    rospy.init_node('node_1',anonymous=True)
    rospy.Subscriber("/cmd_vel",Twist,callback1)
    rospy.spin()

def sub_array():
    rospy.init_node('node_2',anonymous=True)
    rospy.Subscriber("/pub_array",Float32MultiArray,callback2)
    rospy.spin()
def value():
    global data 
    data = [1,2]

def pub_array():
    pub1 = rospy.Publisher('/enc_data',Float32MultiArray,queue_size=10)
    rospy.init_node('enc_array',anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        
        data_ = Float32MultiArray()
        data_.data = [1,2,3,4]
        pub1.publish(data_)
        time.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_array()
    except rospy.ROSInterruptException:
        pass