#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
from rosserial_arduino.srv._Test import *
import numpy as np

class Mic_agv:
    def __init__(self):
        rospy.init_node("agv_mecanum",anonymous=True)
        self.rate = 20
        self.enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,self.cb_enc_value)

        # mecanum wheel
        self.lx = 0.3
        self.ly = 0.3
        self.wheel_diameter = 0.127
        self.encoder_tick_0,self.encoder_tick_1 = 902,1003
        # encoder data
        self.enc_list =[0,0,0,0]
        self.prv_enc_list =[0,0,0,0]
        # pwm data
        self.pwm_list = []
        self.pwm = []
        # infrared data
        # vel
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.th = 0 
        self.imu = 0 # rad/s
        self.dist_list = []
        self.vel_list =[]
        self.vel_t_list =[]
        self.vel_value_list=[]
        # PID
        self.err_list =[0,0,0,0]
        self.prv_err_list = [0,0,0,0]
        self.integral_list = [0,0,0,0]
        self.derivative = [0,0,0,0]
        self.max_pwm = 250
        self.min_pwm = -250
        self.elapsed = None
        self.dis_front = None
        self.dis_rear = None

    
    def cb_enc_value(self,msg):
        self.enc_list = msg.data


    def vel_calculate(self): 
        self.dis_front = (np.array(self.enc_list[0:2])-np.array(self.prv_enc_list[0:2])) / self.encoder_tick_0
        self.dis_rear = (np.array(self.enc_list[2:4])-np.array(self.prv_enc_list[2:4])) / self.encoder_tick_1
        self.dist_list = np.concatenate((self.dis_front,self.dis_rear))
        self.vel_list = self.dist_list / self.elapsed

        self.prv_enc_list = self.enc_list

    def test_sub(self):
        
        now = rospy.Time.now()
        if now > self.t_next:
            self.elapsed = now - self.then
            self.then = now
            self.elapsed = self.elapsed.to_sec()

            # st = rospy.Time.now()
            self.vel_calculate()
            # fn = rospy.Time.now()
            # print((fn-st).to_sec())

            # if(self.vel_list[0] > 0.025):
            #     print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
            print(self.vel_list[0],self.dis_front,self.dis_rear)
            # else:
            #     print(self.vel_list[0])
            
            


    def update(self):
        self.test_sub()
        # self.cmd_cal()
        # self.pid_calculate(5,10,0)#30,400,10
        # print(self.vel_list)
        # print(self.elapsed)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':
    try:
        mic = Mic_agv()
        mic.spin()
    except rospy.ROSInterruptException:
        pass
