
// ##################################################################//
//  Ros version : Noetic
//  Drive type: Mechanum 4wd
//  Date : 12/8/2023
// ##################################################################//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;
#include <DFRobot_WT61PC.h>
#include "SoftwareSerial.h"
SoftwareSerial mySerial(17, 18); // (RX,TX)
DFRobot_WT61PC IMU_sensor(&mySerial);
// ##################################################################//
// Motor control --> [0]->front-l , [1]->front-r ,[2]->rear-l ,[3]->rear-r
const uint8_t DIR[] = {7, 42, 5, 38};
const uint8_t PWM[] = {6, 39, 4, 37};
const uint8_t EA[] = {10, 36, 13, 33};
const uint8_t EB[] = {11, 35, 12, 34};
// IR sensor
const uint8_t ir_sensor[] = {14, 15};
// LED
const uint8_t LED[] = {19, 20, 21}; // R,G,B

// ##################################################################//
// Enc parameter
long int enc_count_0;
long int enc_count_1;
long int enc_count_2;
long int enc_count_3;
long int prv_enc_count[4];
long int enc[4];
// Time parameter
unsigned long prv_time[2];

// Motor parameter
float pwm_value[4];
uint8_t dir[4];

// CMD vel
float vel_x, vel_y, vel_z;
float d_time;
const int TPM[] = {902, 1003}; // tick/meter;
const int pub_rate = 50;       // Hz
float lx = 0.3;                // m
float ly = 0.3;                // m
float wheel_diameter = 0.127;  // m
float vel_act[4];
float vel_flt[4];
float vel_prv[4];
float vel_t[4];
float distance[4];

// PID Control
float prv_err[4];
float err[4];
float I_err[4];
float D_err[4];
float err_p[4];
float KP = 50;
float KI = 10;
float KD = 5;
// ##################################################################//

ros::NodeHandle nh;
// Pub IMU data
std_msgs::Float32 imu_msg;
ros::Publisher IMU("imu_data", &imu_msg);

// Pub Encoder data
std_msgs::Int32MultiArray enc_msg;
ros::Publisher ENC("enc_data", &enc_msg);

// Pub ir data
std_msgs::Int32MultiArray ir_msg;
ros::Publisher IR_sensor("ir_data", &ir_msg);

std_msgs::Float32MultiArray vel_msg;
ros::Publisher VEL("v_data", &vel_msg);

std_msgs::Float32MultiArray vel_t_msg;
ros::Publisher VEL_T("vt_data", &vel_t_msg);

// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);
// ##################################################################//
void gpio_define()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(EA[i], INPUT_PULLUP);
    pinMode(EB[i], INPUT_PULLUP);
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++)
  {
    pinMode(LED[i], OUTPUT);
  }

  for (int i = 0; i < 2; i++)
  {
    pinMode(ir_sensor[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}

void CMD_vel_callback(const geometry_msgs::Twist &CVel)
{
  // Mecanum wheel control
  // invert kinematic
  // w-fl=(vel_x-vel_y-(lx+ly)*vel_z)/r
  // w-fr=(vel_x+vel_y+(lx+ly)*vel_z)/r
  // w-rl=(vel_x+vel_y-(lx+ly)*vel_z)/r
  // w-rr=(vel_x-vel_y+(lx+ly)*vel_z)/r

  // forward kinematic
  // vel_x = (v0+v1+v2+v3)/4;
  // vel_y = (-v0+v1+v2-v3)/4;
  // w_z = (-v0+v1-v2+v3)/(4*(lx+ly));
  
  
  vel_x = CVel.linear.x;   // m/s
  vel_y = CVel.linear.y;   // m/s
  vel_z = CVel.angular.z;  // rad/s

  //  if (abs(vel_z) < 0.02) // prevent sensitive move
  //  {
  //    vel_z = 0.0;
  //  }

  if (vel_x == 0.0 && vel_z == 0.0)
  {
    for (int i = 0; i < 4; i++)
    {
      vel_t[i] = 0;
      vel_flt[i] = 0;
      pwm_value[i] = 0;
      analogWrite(PWM[i], 0);
    }
    pid_clear();
  }
  else
  {
    vel_t[0] = (vel_x - vel_y - (lx + ly) * vel_z);
    vel_t[1] = (vel_x + vel_y + (lx + ly) * vel_z);
    vel_t[2] = (vel_x + vel_y - (lx + ly) * vel_z);
    vel_t[3] = (vel_x - vel_y + (lx + ly) * vel_z);

    for (int i = 0; i < 4; i++)
    {
      if (vel_t[i] / abs(vel_t[i]) > 0)
      {             // +vel_t
        dir[i] = 0; //+
      }
      else
      {
        dir[i] = 1; //-
      }
    }
  }

  // pub vel_t
  float vel_t_value[4] = {vel_t[0], vel_t[1], vel_t[2], vel_t[3]};
  vel_t_msg.data = vel_t_value;
  vel_t_msg.data_length = 4;
  VEL_T.publish(&vel_t_msg);
}

ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMD_vel_callback);

void agv_run(float pwm_[], uint8_t dir_[], uint8_t n)
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(DIR[i], dir_[i]);
    analogWrite(PWM[i], abs(pwm_[i]));
  }
}

void pid(float kp, float ki, float kd, float err_max)
{

  vel_calculate(enc, 4);

  for (int i = 0; i < 4; i++)
  {
    err[i] = vel_t[i] - vel_flt[i];
    err_p[i] = err[i] * 100 / vel_t[i];
    I_err[i] = I_err[i] + err[i] * d_time; // 0.1*0.2
    D_err[i] = (err[i] - prv_err[i]) / d_time;

    for (int i = 0; i < 4; i++)
    {
      if (vel_flt[i] == 0.0)
      {
        I_err[i] = 0.0;
      }
    }

    if (abs(err_p[i]) > err_max)
    {
      pwm_value[i] = kp * err[i] + ki * I_err[i] + kd * D_err[i];
    }
    else
    {
      pwm_value[i] = pwm_value[i];
    }

    prv_err[i] = err[i];

    if (pwm_value[i] > 250)
    {
      pwm_value[i] = 250;
    }
    if (pwm_value[i] < -250)
    {
      pwm_value[i] = -250;
    }
  }
}

void vel_calculate(long int enc_array[], int n)
{

  for (int i = 0; i < 2; i++)
  { // for encoder type 360tpr
    distance[i] = float(enc_array[i] - prv_enc_count[i]) / TPM[0];
    vel_act[i] = (distance[i] / d_time);
  }
  
  for (int i = 2; i < 4; i++)
  { // for encoder type 400tpr
    distance[i] = float(enc_array[i] - prv_enc_count[i]) / TPM[1];
    vel_act[i] = (distance[i] / d_time);
  }
  // Filter velocity at 25 Hz
  for (int i = 0; i < 4; i++)
  {
    vel_flt[i] = 0.854 * vel_flt[i] + 0.0728 * vel_act[i] + 0.0728 * vel_prv[i];
    vel_prv[i] = vel_flt[i];
    prv_enc_count[i] = float(enc_array[i]);
  }

  float vel_value[4] = {vel_flt[0], vel_flt[1], vel_flt[2], vel_flt[3]};
  vel_msg.data = vel_value;
  vel_msg.data_length = 4;
  VEL.publish(&vel_msg);
}

void pid_clear()
{
  for (int i = 0; i < 4; i++)
  {
    prv_err[i] = 0;
    err[i] = 0;
    I_err[i] = 0;
    D_err[i] = 0;
  }
}

void IMU_dfrobot()
{
  if (IMU_sensor.available())
  {
    if (IMU_sensor.Gyro.Z > 250)
    {
      imu_msg.data = (500 - IMU_sensor.Gyro.Z) * 0.017453; // unit rad/s
      IMU.publish(&imu_msg);
    }
    else
    {
      imu_msg.data = -0.017453 * IMU_sensor.Gyro.Z;
      IMU.publish(&imu_msg);
    }
  }
}

void ir_array()
{
  int ir_0 = analogRead(ir_sensor[0] * 0.004882813); // 5/1024
  int ir_1 = analogRead(ir_sensor[1] * 0.004882813); // 5/1024

  int ir_value[] = {ir_0, ir_1};
  ir_msg.data = ir_value;
  ir_msg.data_length = 2;
  IR_sensor.publish(&ir_msg);
}

void enc2array(long int e0, long int e1, long int e2, long int e3)
{
  int enc_value[4] = {e0, e1, e2, e3};
  enc_msg.data = enc_value;
  enc_msg.data_length = 4;
  ENC.publish(&enc_msg);
}

void srv_clear_data(const Test::Request &req, Test::Response &res)
{
  enc_count_0 = 0;
  enc_count_1 = 0;
  enc_count_2 = 0;
  enc_count_3 = 0;
  res.output = "Data has been cleared ";
}

void enc_count0()
{
  if (digitalRead(EB[0]) == 0)
  {
    if (digitalRead(EA[0]) == 0)
    {
      enc_count_0++;
      enc[0] = enc_count_0;
    }
    else
    {
      enc_count_0--;
      enc[0] = enc_count_0;
    }
  }
}

void enc_count1()
{
  if (digitalRead(EB[1]) == 0)
  {
    if (digitalRead(EA[1]) == 0)
    {
      enc_count_1++;
      enc[1] = enc_count_1;
    }
    else
    {
      enc_count_1--;
      enc[1] = enc_count_1;
    }
  }
}

void enc_count2()
{
  if (digitalRead(EB[2]) == 0)
  {
    if (digitalRead(EA[2]) == 0)
    {
      enc_count_2++;
      enc[2] = enc_count_2;
    }
    else
    {
      enc_count_2--;
      enc[2] = enc_count_2;
    }
  }
}

void enc_count3()
{
  if (digitalRead(EB[3]) == 0)
  {
    if (digitalRead(EA[3]) == 0)
    {
      enc_count_3++;
      enc[3] = enc_count_3;
    }
    else
    {
      enc_count_3--;
      enc[3] = enc_count_3;
    }
  }
}

void led_show(char color){
  switch (color){
    case 'r':
    analogWrite(LED[0],HIGH);
    analogWrite(LED[1],LOW);
    analogWrite(LED[2],LOW);
    break;
    case 'g':
    analogWrite(LED[0],LOW);
    analogWrite(LED[1],HIGH);
    analogWrite(LED[2],LOW);
    break;
    case 'y':
    analogWrite(LED[0],LOW);
    analogWrite(LED[1],LOW);
    analogWrite(LED[2],HIGH);
    break;
    default:
    analogWrite(LED[0],LOW);
    analogWrite(LED[1],LOW);
    analogWrite(LED[2],LOW);
    break;
  }
}

void setup()
{
  nh.initNode();
  gpio_define();
  // IMU
  mySerial.begin(9600);
  IMU_sensor.modifyFrequency(FREQUENCY_20HZ);

  // Pub
  nh.advertise(IMU);
  nh.advertise(ENC);       // array
  nh.advertise(VEL);       // array
  nh.advertise(VEL_T);     // array
  nh.advertise(IR_sensor); // array
  // Sub
  nh.subscribe(sub_cmdVel);
  // service
  nh.advertiseService(clear_enc_data);

  enc_count_0 = 0;
  enc_count_1 = 0;
  enc_count_2 = 0;
  enc_count_3 = 0;
}

void loop()
{
  if (vel_t[0] == 0.0 && vel_t[1] == 0.0 && vel_t[2] == 0.0 && vel_t[3] == 0.0)
  { 
    led_show('y');
    for (int i = 0; i < 4; i++)
    {
      analogWrite(PWM[i], 0);
      vel_flt[i] = 0;
      pwm_value[i] = 0;
    }
    pid_clear();
  }
  else
  {
    led_show('g');
    agv_run(pwm_value, dir, 4);
  }

  if (millis() - prv_time[0] >= 50)
  {
    d_time = (millis() - prv_time[0]) * 0.001;
    enc2array(enc_count_0, enc_count_1, enc_count_2, enc_count_3);
    pid(10, 30, 7, 3.0);

    prv_time[0] = millis();
  }

  if (millis() - prv_time[1] >= 100)
  {
    IMU_dfrobot();
    ir_array();

    prv_time[1] = millis();
  }

  nh.spinOnce();
}