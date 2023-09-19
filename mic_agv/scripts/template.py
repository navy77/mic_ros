#!/usr/bin/env python3
import rospy
import roslib
from std_msgs.msg import Int16,String,Float32
from geometry_msgs.msg import Twist

rospy.init_node("pub_simple",anonymous=True)
rate = rospy.Rate(10)

pub1 = rospy.Publisher('string',String,queue_size=10)
pub2 = rospy.Publisher('int16',Int16,queue_size=10)
pub3 = rospy.Publisher('float32',Float32,queue_size=10)
pub4 = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

def talker():
    # pub1.publish("AAA")
    # pub2.publish(123)
    # pub3.publish(0.456)
    x = 0
    while(x<10):
        # msg_cmd = Twist()
        # msg_cmd.linear.x = 0
        # msg_cmd.linear.y = 0
        # msg_cmd.angular.x = 0
        # msg_cmd.angular.z = 1
        # pub4.publish(msg_cmd)
        x+=1
        rospy.loginfo(x)

def callback(sub_data):
    #sub = sub_data.data
    sub = sub_data.linear.y
    rospy.loginfo(sub)
    # print(sub)

def listen():
    # rospy.Subscriber("int16",Int16,callback)
    rospy.Subscriber("cmd_vel",Twist,callback)
def main():
    talker()
    while not rospy.is_shutdown():
        talker()
        #listen()
        rate.sleep()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Error")