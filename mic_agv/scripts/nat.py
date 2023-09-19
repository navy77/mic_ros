#!/usr/bin/env python3
import rospy
import csv
from std_msgs.msg import Int32,Float64,String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from time import *
# MQTT
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

host = '192.168.1.101'
port = 1883
topic = '/ros_mqtt'
client = mqtt.Client()
client.connect(host,port)

tpm = 1698
vel_x = 0.1
vel_z = 0.1

robot_loc = "home"
enc_1 = 0
enc_2 = 0
ir_1 = 0
ir_2 = 0
imu = 0

station = ""
x = 0
y = 0
theta = 0
################################################################################
rospy.init_node("semi_running",anonymous=True)
rate = rospy.Rate(10)
# publish topic
robot_location =  rospy.Publisher("/robot_loc",String,queue_size=10)
cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
################################################################################

def robot_pose_callback(data):
    global robot_loc
    robot_loc = data.data

def encoder_1_callback(data):
    global enc_1
    enc_1 = data.data

def encoder_2_callback(data):
    global enc_2
    enc_2 = data.data

def ir_1_callback(data):
    global ir_1
    ir_1 = data.data

def ir_2_callback(data):
    global ir_2
    ir_2 = data.data

def imu_callback(data):
    global imu
    imu = data.data / 0.017453

# subscribe topic

rospy.Subscriber("/enc_1_value",Int32,encoder_1_callback)
rospy.Subscriber("/enc_2_value",Int32,encoder_2_callback)
rospy.Subscriber("/ir_1_value",Int32,ir_1_callback)
rospy.Subscriber("/ir_2_value",Int32,ir_2_callback)
rospy.Subscriber("/robot_loc",String,robot_pose_callback)
rospy.Subscriber("/imu_value",Float64,imu_callback)

def mqtt_data():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")
    #rospy.loginfo(msg_payload)
    return msg_payload

def read_csv():
    mqtt()
    thisdict = {}
    with open("/home/mic/catkin_ws/src/mic_agv/scripts/location_semi.csv", "r") as csv_file:            
        csv_reader = csv.reader(csv_file,delimiter = ',')            
        for row in csv_reader:
            #print(row)
            thisdict[row[0]] = [row[0], row[1], row[2], row[3]]
    return thisdict

def read_position():  
    dict_position = {}
    dict_position = read_csv()
    #print(dict_position[msg_payload])
    return dict_position[msg_payload]

def goal_position():
    global station,x,y,theta
    station,x,y,theta = read_position()
    station,x,y,theta = str(station),float(x), float(y), float(theta)
    #rospy.loginfo(msg_payload)
    return station,x,y,theta

def stop_move():
    msg_cmd = Twist()
    msg_cmd.linear.x = 0
    msg_cmd.angular.z = 0
    cmd_vel.publish(msg_cmd)
    
def move_straight(distance):
    target_tick = tpm * abs(distance)
    rospy.loginfo(target_tick)
    rospy.Subscriber("/enc_1_value",Int32,encoder_1_callback) # for set init encoder 
    enc_init = enc_1
    target_tick = target_tick+enc_init
    rospy.loginfo("encoder_init : %s"%enc_1)
    rospy.loginfo("encoder_target : %s"%target_tick)
    msg_cmd = Twist()
    msg_cmd.linear.x = vel_x*(distance/abs(distance))
    
    while(enc_1<target_tick):
        rospy.Subscriber("/enc_1_value",Int32,encoder_1_callback)
        rospy.Subscriber("/ir_1_value",Int32,ir_1_callback)
        rospy.Subscriber("/ir_2_value",Int32,ir_2_callback)
        rospy.Subscriber("/imu_value",Float64,imu_callback)
        if imu > abs(5):
            stop_move()
            sleep(0.1)
            move_turn(imu)
            sleep(0.1)
        # if ir_1 <7 or ir_2 <7:
        #     stop_run()
        #sleep(0.1)
        cmd_vel.publish(msg_cmd)
        rospy.loginfo(enc_1)
    stop_move()

def move_turn(degree):
    cons_time = 0.01745/vel_z
    target_rotate = abs(degree)*cons_time  #time [s]
    rospy.loginfo(target_rotate)
    msg_cmd = Twist()
    msg_cmd.angular.z = vel_z*(degree/abs(degree))

    cmd_vel.publish(msg_cmd)
    sleep(target_rotate)
    stop_move()

def program_1():
    global robot_loc
    rospy.loginfo("start program_1")
    sleep(1)
    move_straight(1.5)
    move_straight(10)
    #move_turn(-90)
    rospy.loginfo("finish program_1")
    robot_loc = "home"
    client.publish('/agv_feedback','Finished!')

def program_2():
    global robot_loc
    rospy.loginfo("start program_2")
    sleep(1)
    #move_straight(0.1)
    move_turn(90)
    rospy.loginfo("finish program_2")
    robot_loc = "home"
    client.publish('/agv_feedback','Finished!')

def program_3():
    global robot_loc
    rospy.loginfo("start program_3")
    sleep(1)
    #move_straight(0.1)
    move_turn(90)
    rospy.loginfo("finish program_3")
    robot_loc = "home"
    client.publish('/agv_feedback','Finished!')

def select_program():
    if str(robot_loc) =="line_1":
        robot_location.publish(robot_loc)
        program_1()
    elif str(robot_loc) =="line_2":
        robot_location.publish(robot_loc)
        program_2()
    elif str(robot_loc) =="line_3":
        robot_location.publish(robot_loc)
        program_3()

def program_feed():
    pass

def main():
    global robot_loc
    
    while not rospy.is_shutdown(): 
        robot_location.publish(robot_loc)

        if str(robot_loc) == "home": #check robot standby at home position
            rospy.loginfo(robot_loc)
            mqtt_data()     
            client.publish('/agv_feedback','Received!')
            robot_loc = str(msg_payload)
            rospy.loginfo(robot_loc)
            select_program()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("Error!!!")

   