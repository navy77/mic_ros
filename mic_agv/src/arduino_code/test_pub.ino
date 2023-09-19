#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include <ros/time.h>
int a,b,c,d;
long prv_time;

ros::NodeHandle nh;

// Pub Encoder data
std_msgs::Int32MultiArray enc_msg;
ros::Publisher ENC("enc_data", &enc_msg);

std_msgs::Float32 imu_msg;
ros::Publisher IMU("imu_data", &imu_msg);

void setup() {
  nh.initNode();
  nh.advertise(ENC);
  nh.advertise(IMU);
//  Serial.begin(115200);

}

void loop() {
  if(millis()-prv_time >=50){
    a = millis()/50;

    int enc[4] = {a,a,a,a};
    enc_msg.data = enc;
    enc_msg.data_length = 4;
    ENC.publish(&enc_msg);

    float imu = 0.1;
    imu_msg.data = imu;
    IMU.publish(&imu_msg);

    a++;
    prv_time = millis();
  }
  
  nh.spinOnce();
}