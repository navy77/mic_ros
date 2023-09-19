//##################################################################//
// Autonomous with NMB Stepping motor
// Motor model: GT0200-23M26617 ,step/rev:400 ,gear ratio: 1:20
// Date 5 MAY 2023,rev.A
// Dev.Board : Cytron maker PI RP2040
//##################################################################//
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;

#include <AccelStepper.h>
#include <Wire.h>
#include <L3G.h>
//##################################################################//
//Motor control
// Left
const uint8_t PulPin_1 = 9; // pulse pin
const uint8_t DirPin_1 = 8; // direction pin
long int encoder_1 =0; // encoder count
// Right
const uint8_t PulPin_2 = 1; // pulse pin
const uint8_t DirPin_2 = 0; // direction pin
long int encoder_2 =0; // encoder count
// CMD_vel
float vel_x; // linear velocity
float vel_z; // augular velocity
float vel_1; // vel left wheel
float vel_2; // vel right wheel
// Time loop
unsigned long prv_time1 = 0;
unsigned long prv_time2 = 0;
unsigned long prv_time3 = 0;
unsigned long prv_time4 = 0;
// Stepping motor
const int SPR = 400; // motor step peer revolution
const int max_speed = 2000; // motor max speed [unit=sps]
const int gear_ratio = 20; // motor gear ratio
int SPS_value_1; // motor left step per second 
int SPS_value_2; // motor right step per second
int RPM_value_1; // motor left round per minute
int RPM_value_2; // motor right round per minute 
// Diff drive control
float base_width = 0.485; // distance between center of wheel left-->right
float wheel_diameter = 0.15; //wheel diameter
// IR
const uint8_t IR_pin1 = 26;
const uint8_t IR_pin2 = 27;
int ir_1; // ir sensor 1
int ir_2; // ir sensor 2
// IMU
const uint8_t SDA_pin = 4;
const uint8_t SCL_pin = 5;
//RGB
const uint8_t ledR = 6;
const uint8_t ledG = 7;

//##################################################################//
AccelStepper stepper_1 = AccelStepper(1, PulPin_1, DirPin_1);
AccelStepper stepper_2 = AccelStepper(1, PulPin_2, DirPin_2);
L3G gyro;
ros::NodeHandle  nh;
// Pub IMU data
std_msgs::Float64 imu_msg;
ros::Publisher IMU("imu_value", &imu_msg);

// Pub Encoder data
std_msgs::Int32 enc_1_msg;
ros::Publisher ENC_1("enc_1_value", &enc_1_msg);

std_msgs::Int32 enc_2_msg;
ros::Publisher ENC_2("enc_2_value", &enc_2_msg);

std_msgs::Int32 sps_1_msg;
ros::Publisher SPS_1("sps_1_value", &sps_1_msg);

std_msgs::Int32 sps_2_msg;
ros::Publisher SPS_2("sps_2_value", &sps_2_msg);

std_msgs::Int32 ir_1_msg;
ros::Publisher IR_1("ir_1_value", &ir_1_msg);

std_msgs::Int32 ir_2_msg;
ros::Publisher IR_2("ir_2_value", &ir_2_msg);

void srv_clear_data(const Test::Request &req, Test::Response &res);
// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);
void pin_def0(){
  // IMU i2c define pin
  Wire.setSDA(SDA_pin);
  Wire.setSCL(SCL_pin);
  Wire.begin();
  //  IR sensor analog
  pinMode(IR_pin1,INPUT);//analog gpio26,27,28
  pinMode(IR_pin2,INPUT);
  // RGB
  pinMode(ledR,OUTPUT);
  pinMode(ledG,OUTPUT);
}
void pin_def1(){
    // motor_left
    pinMode(PulPin_1, OUTPUT);
    pinMode(DirPin_1, OUTPUT);
    stepper_1.setMaxSpeed(max_speed);
    // motor_right
    pinMode(PulPin_2, OUTPUT);
    pinMode(DirPin_2, OUTPUT);
    stepper_2.setMaxSpeed(max_speed);
}
// cmd_vel
void CMDvelCallback(const geometry_msgs::Twist &CVel)
{
    vel_x = CVel.linear.x;
    vel_z = CVel.angular.z;

    if (abs(vel_z) < 0.05)
    {
        vel_z = 0;
    }
    if (vel_x == 0.0 && abs(vel_z) > 0){
    vel_1 = -vel_z * base_width*1.0;
    vel_2 = -1*vel_1;
    }
    else{
    vel_1 = vel_x - vel_z * base_width / 2;
    vel_2 = vel_x + vel_z * base_width / 2;
      }
}
ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMDvelCallback);

void srv_clear_data(const Test::Request &req, Test::Response &res)
{   
    stepper_1.setCurrentPosition(0);
    stepper_2.setCurrentPosition(0);
    enc_1_msg.data = 0;
    enc_2_msg.data = 0;
    encoder_1 = 0;
    encoder_2 = 0;
    res.output = "Data has been cleared ";
}
void setup() {
    nh.initNode();
    pin_def0();
    // publish
    nh.advertise(IMU);
    nh.advertise(SPS_1);
    nh.advertise(SPS_2);
    nh.advertise(ENC_1);
    nh.advertise(ENC_2);
    nh.advertise(IR_1);
    nh.advertise(IR_2);
    // service
    nh.advertiseService(clear_enc_data);
    // sub
    nh.subscribe(sub_cmdVel);
    stepper_1.setCurrentPosition(0);
    stepper_2.setCurrentPosition(0);
    enc_1_msg.data = 0;
    enc_2_msg.data = 0;
    // IMU:L3G4200D
    gyro.init();
    gyro.enableDefault();
    // light
    Red_light();
}
void setup1(){
    pin_def1();
}
void imu_sensor(){//L3G
  gyro.read();
  imu_msg.data = gyro.g.z * 0.00875 * 0.017453;
  //imu_msg.data = gyro.g.z;
  IMU.publish(&imu_msg);
}
void ir_sensor(){
  ir_1 = 2076/(analogRead(IR_pin1)-11);
  ir_2 = 2076/(analogRead(IR_pin2)-11);
  
  ir_1_msg.data = ir_1;
  ir_2_msg.data = ir_2;
  IR_1.publish(&ir_1_msg);
  IR_2.publish(&ir_2_msg);
}
void SPS_calculate(float vel_1, float vel_2)
{   
//    if(ir_1 > 15 || ir_2 > 15){
//    RPM_value_1 = 0;
//    RPM_value_2 = 0;
//    Red_light();
//    }
//    else{
    Green_light();
    // calculate RPM
    RPM_value_1 = (-60 * vel_1)/(PI*wheel_diameter);
    RPM_value_2 = (-60 * vel_2)/(PI*wheel_diameter);
    //}
    // calculate SPS
    SPS_value_1 = (RPM_value_1 * SPR * gear_ratio)/60;
    SPS_value_2 = (RPM_value_2 * SPR * gear_ratio)/60;
    
    // vel_1 left
    if (SPS_value_1 > max_speed)
    {
        SPS_value_1 = max_speed;
    }
    else if (SPS_value_1 < max_speed * (-1))
    {
        SPS_value_1 = max_speed * (-1);
    }
    else {
      SPS_value_1= SPS_value_1;
    }
    // vel_2 right
    if (SPS_value_2 > max_speed)
    {
        SPS_value_2 = max_speed;
    }
    else if (SPS_value_2 < max_speed * (-1))
    {
        SPS_value_2 = max_speed * (-1);
    }
    else {
      SPS_value_2= SPS_value_2;
    }

    sps_1_msg.data = SPS_value_1;
    SPS_1.publish(&sps_1_msg);

    sps_2_msg.data = SPS_value_2;
    SPS_2.publish(&sps_2_msg);

}
void Red_light(){
  analogWrite(ledR,255);
  analogWrite(ledG,0);
}
void Green_light(){
  analogWrite(ledG,255);
  analogWrite(ledR,0);
}
void loop(){
  nh.spinOnce();
  if(abs(stepper_1.currentPosition())>2147483000 || abs(stepper_2.currentPosition())>2147483000){
    stepper_1.setCurrentPosition(0);
    stepper_2.setCurrentPosition(0);
  }
  
  if (millis()-prv_time4 >500){
    ir_sensor();
    prv_time4 = millis();
  }
  // encoder publish at 10 hz
  if (millis() - prv_time1 > 100){
    enc_1_msg.data = stepper_1.currentPosition()/-10;
    enc_2_msg.data = stepper_2.currentPosition()/-10;
    ENC_1.publish(&enc_1_msg);
    ENC_2.publish(&enc_2_msg);
    prv_time1 = millis();
  }
   //IMU publish at 20 hz
  if (millis()-prv_time2 >50){
    imu_sensor();
    prv_time2 = millis();
  }
  // motor publish at 5 hz
  if (millis()-prv_time3 >200){
    SPS_calculate(vel_1, vel_2);
    prv_time3 = millis();
  }
}
void loop1(){
  stepper_1.setSpeed(SPS_value_1);
  stepper_2.setSpeed(SPS_value_2);
  stepper_1.runSpeed();
  stepper_2.runSpeed();
}
