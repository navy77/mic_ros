#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
from rosserial_arduino.srv._Test import *
import numpy as np

class Mic_agv:
    def __init__(self):
        rospy.init_node("agv_mecanum",anonymous=True)
        self.rate = 10
        self.pwm_value = rospy.Publisher('/pwm_data',Float32MultiArray,queue_size=10)
        # self.vel_value = rospy.Publisher('/vel_data',Float32MultiArray,queue_size=10)
        self.cmd_value = rospy.Subscriber('/cmd_vel',Twist,self.cb_cmd_vel)
        self.enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,self.cb_enc_value)
        self.ir_value = rospy.Subscriber('/ir_data',Int32MultiArray,self.cb_ir_value)
        self.imu_value = rospy.Subscriber('/imu_data',Float32,self.cb_imu_value)
        # cmd_vel
        self.vel_x,self.vel_y,self.vel_z = 0,0,0
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
        self.ir_list = []
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

    def cb_imu_value(self,msg):
        self.imu = msg.data

    def cb_cmd_vel(self,Twist):
        self.vel_x = Twist.linear.x
        self.vel_y = Twist.linear.y
        self.vel_z = Twist.angular.z
    
    def cb_enc_value(self,msg):
        self.enc_list = msg.data

    def cb_ir_value(self,msg):
        self.ir_list = msg.data
        # self.ir_0 = msg.data[0]
        # self.ir_1 = msg.data[1]

    def cmd_cal(self):
        if self.vel_x == 0 and self.vel_y == 0 and self.vel_z == 0:
            # self.vel_list = [0,0,0,0]
            self.vel_t_list = [0,0,0,0]
            self.pwm_list = [0,0,0,0]
        else:    
            vel_t_0 = self.vel_x-self.vel_y-(self.lx+self.ly)*self.vel_z
            vel_t_1 = self.vel_x+self.vel_y+(self.lx+self.ly)*self.vel_z 
            vel_t_2 = self.vel_x+self.vel_y-(self.lx+self.ly)*self.vel_z
            vel_t_3 = self.vel_x-self.vel_y+(self.lx+self.ly)*self.vel_z
            self.vel_t_list = [vel_t_0,vel_t_1,vel_t_2,vel_t_3]

    def low_pass_filter(self):
        pass

    def vel_calculate(self): 
        dis_front = (np.array(self.enc_list[0:2])-np.array(self.prv_enc_list[0:2])) / self.encoder_tick_0
        dis_rear = (np.array(self.enc_list[2:4])-np.array(self.prv_enc_list[2:4])) / self.encoder_tick_1
        self.dist_list = np.concatenate((dis_front,dis_rear))
        self.vel_list = np.round(self.dist_list/self.elapsed,3)

        self.prv_enc_list = self.enc_list

        return self.vel_list 
    
    def pid_calculate(self,kp,ki,kd):
        
        now = rospy.Time.now()
        if now > self.t_next:

            self.elapsed = now - self.then
            self.then = now
            self.elapsed = self.elapsed.to_sec()

            self.vel_calculate()

        #     self.err_list = np.array(self.vel_t_list) - np.array(self.vel_list)
        #     # Proportional term
        #     p = kp * self.err_list
        #     # Integral term
        #     self.integral_list = np.array(self.integral_list) + np.array(self.err_list)*self.elapsed
        #     i = ki * self.integral_list
        #     # Derivative term
        #     self.derivative = (np.array(self.err_list)-np.array(self.prv_err_list))/self.elapsed
        #     d = kd * self.derivative

        #     # anti windup error
        #     self.wind_err(5)
  
        #     self.pwm = p + i + d

        #     for j in range(4):
        #         if self.vel_t_list[j] == 0:
        #             self.pwm[j] = 0

        #     for j in range(4):
        #         if self.pwm[j]>self.max_pwm:
        #             self.pwm[j] = self.max_pwm

        #     for j in range(4):
        #         if self.pwm[j]<self.min_pwm:
        #             self.pwm[j] = self.min_pwm   
            
        # self.pwm_list = Float32MultiArray()
        # self.pwm_list.data = self.pwm

        # self.prv_err_list = self.err_list
        # self.pwm_value.publish(self.pwm_list)

    def wind_err(self,max_ierr):
        for j in range(4):
            if abs(self.integral_list[j]) > max_ierr:
                self.integral_list[j] = 0

        return self.integral_list 

    def update(self):
        self.cmd_cal()
        self.pid_calculate(5,10,0)#30,400,10
        print(self.vel_list)
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
