#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
import csv
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt

host = '192.168.1.101'
port = 1883
topic = '/ros_mqtt'
time_out = 120

global move_base,client
client = mqtt.Client()
client.connect(host, port)
rospy.init_node('nav_goal', anonymous=False)
move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
# rospy.on_shutdown(shutdown())
rate = rospy.Rate(5) # 5 hz
rospy.loginfo("wait for the action server to come up")
move_base.wait_for_server(rospy.Duration(5))

def mqtt():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")
    rospy.loginfo(msg_payload)
    # client.publish('/agv_feedback','OK_GO')
    
def read_csv():
    thisdict = {}
    with open("/home/mic/catkin_ws/src/mic_agv/scripts/location_nat5.csv", "r") as csv_file:            
        csv_reader = csv.reader(csv_file,delimiter = ',')
        for row in csv_reader:
            #print(row)
            thisdict[row[0]] = [row[1], row[2], row[3]]
    return thisdict

def read_position():  
    dict_position = {}
    dict_position = read_csv()
    rospy.loginfo(dict_position[msg_payload])
    return dict_position[msg_payload]

def shutdown():
    stop_goal = MoveBaseGoal()
    move_base.send_goal(stop_goal)
    rospy.loginfo("Stop")

def movebase_client():
    x,y,theta = read_position()
    x,y,theta = float(x), float(y), float(theta)
    #convert euler to quanternion
    q = quaternion_from_euler(0,0,theta)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))

    move_base.send_goal(goal)
    wait = move_base.wait_for_result(rospy.Duration(time_out))
    state = move_base.get_state()
    result = False 
    if wait and state == GoalStatus.SUCCEEDED:
        result = True 
        client.publish('/agv_feedback','Succeeded')
    else:
        move_base.cancel_goal()
    return result

def main():
    while not rospy.is_shutdown():
        mqtt()
        movebase_client()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
