#!/usr/bin/env python3
import rospy
from math import sin,cos,pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32,Float32,Int32MultiArray

class Mic_ros:
    def __init__(self):
        rospy.init_node("agv_mecanum",anonymous=True)
        self.rate = 2
        self.enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,self.cb_enc_value)
        self.imu_value = rospy.Subscriber('/imu_data',Float32,self.cb_imu_value)
        # mecanum wheel
        self.lx = 0.3
        self.ly = 0.3
        self.wheel_diameter = 0.127
        self.encoder_tick_0,self.encoder_tick_1 = 902,1003
        # encoder data
        self.enc_0 = 0
        self.enc_1 = 0
        self.enc_2 = 0
        self.enc_3 = 0

        self.prv_enc_0 = 0
        self.prv_enc_1 = 0
        self.prv_enc_2 = 0
        self.prv_enc_3 = 0

        self.th = 0 
        self.imu = 0 # rad/s
        # odom
        self.th = 0 
        self.x = 0
        self.y = 0
        self.base_frame_id = "base_link"
        self.odom_frame_id = "odom"

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
    
    def cb_enc_value(self,msg):
        # self.enc_list = msg.data
        self.enc_0 = msg.data[0]
        self.enc_1 = msg.data[1]
        self.enc_2 = msg.data[2]
        self.enc_3 = msg.data[3]

    def cb_imu_value(self,msg):
        self.imu = msg.data # unit rad/s

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.enc_0 == None:
                d_0 = 0
                d_1 = 0
                d_2 = 0
                d_3 = 0
            else:
                d_0 = (self.enc_0-self.prv_enc_0)/self.encoder_tick_0
                d_1 = (self.enc_1-self.prv_enc_1)/self.encoder_tick_0
                d_2 = (self.enc_2-self.prv_enc_2)/self.encoder_tick_1
                d_3 = (self.enc_3-self.prv_enc_3)/self.encoder_tick_1

            self.prv_enc_0 = self.enc_0
            self.prv_enc_1 = self.enc_1
            self.prv_enc_2 = self.enc_2
            self.prv_enc_3 = self.enc_3

            th = self.imu
            
            deltaXTravel = (d_0 + d_1 + d_2 + d_3)/4.0
            deltaYTravel = (-d_0 + d_1 + d_2 - d_3)/4.0
            deltaTheta = self.imu

            pos_x = pos_x + deltaXTravel*cos(th)
            # deltaXTravel = (frontLeftTravel + frontRightTravel + rearLeftTravel + rearRightTravel) / 4.0
            # deltaYTravel = (-frontLeftTravel + frontRightTravel + rearLeftTravel - rearRightTravel) / 4.0
            # deltaTheta = (-frontLeftTravel + frontRightTravel - rearLeftTravel + rearRightTravel) / (2 * (self.wheelSeparation + self.wheelSeparationLength))
            print(deltaXTravel,"::",deltaYTravel,"::",deltaTheta)

    def spin(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    try:
        mic = Mic_ros()
        mic.spin()
    except rospy.ROSInterruptException:
        pass