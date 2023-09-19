#!/usr/bin/env python3
import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32,Float64

rospy.init_node('diff_drive', anonymous=False)
#### parameters #######
rate = rospy.Rate(10)
ticks_meter_left = float(1062) 
ticks_meter_right = float(1062)
base_width = float(0.350)
base_frame_id = 'base_link'
odom_frame_id = 'odom'
encoder_min = -2147483647
encoder_max = 2147483647
time_delta = rospy.Duration(1.0/rate)
t_next = rospy.Time.now() + time_delta

enc_left = None
enc_right = None
d_left = 0     
d_right = 0
prv_enc_left = 0
prv_enc_right = 0
ang_z = 0
imu = 0
prv_ang_z = 0
x = 0                  # position in xy plane 
y = 0
th = 0
dx = 0                 # speeds in x/rotation
dr = 0
then = rospy.Time.now()

# subscriptions
odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
odomBroadcaster = TransformBroadcaster()

def wheel_left_callback(enc_left_msg):
    global enc_left,prv_enc_left
    enc_left = enc_left_msg.data
    prv_enc_left = enc_left

def wheel_left_callback(enc_right_msg):
    global enc_right,prv_enc_right
    enc_right = enc_right_msg.data
    prv_enc_right = enc_right

def imu_callback(imu_msg):
    global imu,prv_ang_z
    imu = imu_msg.data
    prv_ang_z = imu * 0.8

def update():
    rospy.Subscriber("enc_left_value", Int32,wheel_left_callback)
    rospy.Subscriber("enc_right_value", Int32,wheel_left_callback)
    rospy.Subscriber("imu_value", Float64, imu_callback)
    
    now = rospy.Time.now()
    if now > t_next:
        elapsed = now - then
        then = now
        elapsed = elapsed.to_sec()

        # calculate odometry
        if enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (d_left - enc_left) / ticks_meter_left
            d_right = (d_right - enc_right) / ticks_meter_right
            
        enc_left = d_left
        enc_right = d_right
            
        # distance traveled is the average of the two wheels 
        d = ( d_left + d_right ) / 2
        # this approximation works (in radians) for small angles
        # th = ( d_right - d_left ) / base_width
        # th = prev_angZ * 0.0174532925 # convert to radian
        th = prv_ang_z
        # calculate velocities
        dx = d / elapsed
        dr = th 
                     
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            x = x + ( cos( th ) * x - sin( th ) * y )
            y = y + ( sin( th ) * x + cos( th ) * y )
        
        #if (abs(th) = 0.01):
        if (th != 0):
            th = th + th
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( th / 2 )
            quaternion.w = cos( th / 2 )
            odomBroadcaster.sendTransform(
                (x, y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                base_frame_id,
                odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = odom_frame_id
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = base_frame_id
            odom.twist.twist.linear.x = dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = dr
            odomPub.publish(odom)

def main():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
