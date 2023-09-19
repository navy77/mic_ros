#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
import time
from rosserial_arduino.srv._Test import *

# cmd_vel
vel_x,vel_y,vel_z = 0,0,0
# mecanum_wheel
lx,ly,wheel_diameter = 0.3,0.3,0.127
encoder_tick_0,encoder_tick_1 = 2855,3150
# encoder data
enc_0,enc_1,enc_2,enc_3 = 0,0,0,0
prv_enc_0,prv_enc_1,prv_enc_2,prv_enc_3 = 0,0,0,0
# pwm data
pwms = []
pwm_0,pwm_1,pwm_2,pwm_3 = 0,0,0,0
# infrared data
ir_0,ir_1 = 0,0
# vel
# t_delta = rospy.Duration(1.0/rate)
# t_next = rospy.Time.now() + t_delta
# then = rospy.Time.now()
vel_0,vel_1,vel_2,vel_3 = 0,0,0,0
theta  = 0
imu = 0 # rad/s

def cb_cmd_vel(Twist):
    vel_x = Twist.linear.x
    vel_y = Twist.linear.y
    vel_z = Twist.angular.z
    # print(vel_x)

def cb_enc_value(msg):
    global enc_0,enc_1,enc_2,enc_3
    enc_0 = msg.data[0]
    enc_1 - msg.data[1]
    enc_2 = msg.data[2]
    enc_3 - msg.data[3] 
    print(enc_0)
    
def cb_ir_value(msg):
    global ir_0,ir_1
    ir_0 = msg.data[0]
    ir_1 = msg.data[1]

def cb_imu_value(msg):
    global imu
    imu = msg.data    

def update():
    pass

def main():
    global rate
    rospy.init_node('mecanum_wheel',anonymous=True)
    # pwm_value = rospy.Publisher('/pwm_data',Float32MultiArray,queue_size=10)
    cmd_value = rospy.Subscriber('/cmd_vel',Twist,cb_cmd_vel)
    enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,cb_enc_value)
    # ir_value = rospy.Subscriber('/ir_data',Int32MultiArray,cb_ir_value)
    # imu_value = rospy.Subscriber('/imu_data',Float32,cb_imu_value)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        update()
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass