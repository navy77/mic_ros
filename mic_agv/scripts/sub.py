#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float64,Float32MultiArray

import time

enc_list = []

def callback2(data):
    enc_list=data.data
    rospy.loginfo(enc_list)


def sub_array():
    rospy.init_node('node_1',anonymous=True)
    rospy.Subscriber("/enc_data",Float32MultiArray,callback2)
    rospy.spin()


if __name__ == '__main__':
    try:
        sub_array()
    except rospy.ROSInterruptException:
        pass