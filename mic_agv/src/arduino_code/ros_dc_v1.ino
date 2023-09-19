// ##################################################################//
//  Autonomous with DC motor
//  Note: DC motor 24v + encoder 400
// Date 12 Jan 2023

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;

#include <DFRobot_WT61PC.h>
#include "SoftwareSerial.h"
SoftwareSerial mySerial(17, 18); // (RX,TX)
DFRobot_WT61PC IMU_sensor(&mySerial);

// IMU buildin
Adafruit_MPU6050 mpu;
const uint8_t SCLpin = 40;
const uint8_t SDApin = 41;
// Tower light
const uint8_t RED = 13;
const uint8_t YELLOW = 14;
const uint8_t GREEN = 15;
const uint8_t ALARM = 16;
// Motor control
// Left
const uint8_t DIR_1 = 36;
const uint8_t PWM_1 = 37;
// Right
const uint8_t DIR_2 = 38;
const uint8_t PWM_2 = 39;
// Encoder control
// Left
const uint8_t E1A = 4;
const uint8_t E1B = 5;
// Right
const uint8_t E2A = 6;
const uint8_t E2B = 7;

long int count_l = 0;
long int count_r = 0;
float prv_count_l = 0;
float prv_count_r = 0;

// CMD_vel
float vel_x;
float vel_z;
int DIR1, DIR2;

float vel_left;
float vel_left_act;
float vel_left_flt = 0;
float vel_letf_prv = 0;

float vel_right;
float vel_right_act;
float vel_right_flt = 0;
float vel_right_prv = 0;

float d_time;
unsigned long prv_time1 = 0;
unsigned long prv_time2 = 0;
const int TPM = 1062;    // tick per meter
const int pub_rate = 50; // hz

// PID
float prv_error_left = 0;
float prv_error_right = 0;

float error_left = 0;
float error_right = 0;

float I_error_left = 0;
float I_error_right = 0;

float D_error_left = 0;
float D_error_right = 0;

float KP = 50; // 500
float KI = 10; // 100
float KD = 5;  // 50

// Motor control
int PWM_left_value;
int PWM_right_value;
float base_width = 0.445;

//**********************************************************************************//
ros::NodeHandle nh;

// Pub IMU data
std_msgs::Float64 imu_msg;
ros::Publisher IMU("imu_value", &imu_msg);

// Pub Encoder data
std_msgs::Int32 enc_left_msg;
ros::Publisher ENC_LEFT("enc_left_value", &enc_left_msg);

std_msgs::Int32 enc_right_msg;
ros::Publisher ENC_RIGHT("enc_right_value", &enc_right_msg);

// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Pub vel data
std_msgs::Float64 vel_left_msg;
ros::Publisher VEL_LEFT("vel_left_value", &vel_left_msg);

std_msgs::Float64 vel_right_msg;
ros::Publisher VEL_RIGHT("vel_right_value", &vel_right_msg);

// Pub pwm data
std_msgs::Int32 pwm_left_msg;
ros::Publisher PWM_LEFT("pwm_left_value", &pwm_left_msg);

std_msgs::Int32 pwm_right_msg;
ros::Publisher PWM_RIGHT("pwm_right_value", &pwm_right_msg);

// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);

void pin_define()
{
    // encoder l
    pinMode(E2A, INPUT_PULLUP);
    pinMode(E2B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(E2A), countL, CHANGE);
    // encoder r
    pinMode(E1A, INPUT_PULLUP);
    pinMode(E1B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(E1A), countR, CHANGE);

    // Motor control L
    pinMode(DIR_1, OUTPUT);
    pinMode(PWM_1, OUTPUT);
    // Motor control R
    pinMode(DIR_2, OUTPUT);
    pinMode(PWM_2, OUTPUT);

    pinMode(RED, OUTPUT);
    pinMode(YELLOW, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(ALARM, OUTPUT);
}

// cmd_vel
void CMDvelCallback(const geometry_msgs::Twist &CVel)
{
    vel_x = CVel.linear.x;
    vel_z = CVel.angular.z;

    if (abs(vel_z) < 0.02)
    {
        vel_z = 0.0;
    }

    if (vel_x == 0.0 && vel_z == 0.0)
    {
        vel_left = 0.0;
        vel_right = 0.0;

        analogWrite(PWM_1, 0);
        analogWrite(PWM_2, 0);

        pid_clear();
    }
    else if (vel_x == 0.0) // turn left,right
    {
        if (vel_z > 0.0)
        {
            DIR1 = 1; //Left motor
            DIR2 = 0; //Right motor
        }
        else
        {
            DIR1 = 0; //Left motor
            DIR2 = 1; //Right motor
        }
        vel_right = vel_z * 1; // 0.5
        vel_left = -1 * vel_right;
    }
    else // move straight,arc
    {
        if (vel_x > 0.0)
        {
            DIR1 = 1;
            DIR2 = 1;
        }
        else
        {
            DIR1 = 0;
            DIR2 = 0;
            vel_left = vel_right = vel_x;
        }
        vel_left = vel_x - vel_z * base_width / 2.0;
        vel_right = vel_x + vel_z * base_width / 2.0;
    }

    //    vel_left = vel_x - vel_z * base_width /2.0;
    //    vel_right = vel_x + vel_z * base_width /2.0;
}

ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMDvelCallback);

void setup()
{
    nh.initNode();
    pin_define();
    Wire.begin(SDApin, SCLpin);
    mpu.begin();
    delay(1000);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2,4,8,16

    mpu.setGyroRange(MPU6050_RANGE_250_DEG); // 250,500,1000,2000

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // 260,184,94,44,21,10,5

    // IMU
        Serial.begin(57600);
        mySerial.begin(9600);
        IMU_sensor.modifyFrequency(FREQUENCY_20HZ);

    // pub
    nh.advertise(IMU);
    nh.advertise(ENC_LEFT);
    nh.advertise(ENC_RIGHT);
    nh.advertise(VEL_LEFT);
    nh.advertise(VEL_RIGHT);
    nh.advertise(PWM_LEFT);
    nh.advertise(PWM_RIGHT);

    // service
    nh.advertiseService(clear_enc_data);

    // sub
    nh.subscribe(sub_cmdVel);

    enc_left_msg.data = 0;
    enc_right_msg.data = 0;
}

void loop()
{
    nh.spinOnce();
    if (vel_left == 0.0 || vel_right == 0.0)
    {
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, HIGH);
        digitalWrite(GREEN, LOW);
        //      digitalWrite(ALARM,HIGH);
    }
    else
    {
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, HIGH);
        //      digitalWrite(ALARM,HIGH);
    }

    if (millis() - prv_time1 > (1000 / pub_rate))
    {
//        imu();
        imu_df();
        ENC_LEFT.publish(&enc_left_msg);
        ENC_RIGHT.publish(&enc_right_msg);

        vel_calculate(count_l, count_r);

        pid(vel_left, vel_left_flt, vel_right, vel_right_flt);

        prv_time1 = millis();
    }
    agv_run(DIR1, DIR2, PWM_left_value, PWM_right_value);
}

void vel_calculate(float count_l, float count_r)
{
    float d_time = millis() - prv_time2;

    float d_left = float(count_l - prv_count_l) / TPM;
    vel_left_act = (d_left / d_time) * 1000;

    float d_right = float(count_r - prv_count_r) / TPM;
    vel_right_act = (d_right / d_time) * 1000;

    // filter vel at 25 Hz
    vel_left_flt = 0.854 * vel_left_flt + 0.0728 * vel_left_act + 0.0728 * vel_letf_prv;
    vel_letf_prv = vel_left_flt;

    // filter vel at 25 Hz
    vel_right_flt = 0.854 * vel_right_flt + 0.0728 * vel_right_act + 0.0728 * vel_right_prv;
    vel_right_prv = vel_right_flt;

    prv_count_l = float(count_l);
    prv_count_r = float(count_r);

    prv_time2 = millis();

    vel_left_msg.data = vel_left_flt;
    vel_right_msg.data = vel_right_flt;

    VEL_LEFT.publish(&vel_left_msg);
    VEL_RIGHT.publish(&vel_right_msg);
}

void pid(float vel_left, float vel_left_flt, float vel_right, float vel_right_flt)
{

    error_left = vel_left - vel_left_flt;
    error_right = vel_right - vel_right_flt;

    I_error_left = I_error_left + error_left;
    I_error_right = I_error_right + error_right;

    D_error_left = error_left - prv_error_left;
    D_error_right = error_right - prv_error_right;

    if (vel_left == 0.0 && error_left == 0.0)
    {
        I_error_left = 0;
    }

    if (vel_right == 0.0 && error_right == 0.0)
    {
        I_error_right = 0;
    }

    if (error_left > 0.6 || error_right > 0.6)
    {
        pid_clear();
    }

    PWM_left_value = (KP * error_left) + (KI * I_error_left) + (KD * D_error_left);
    PWM_right_value = (KP * error_right) + (KI * I_error_right) + (KD * D_error_right);

    prv_error_left = error_left;
    prv_error_right = error_right;

    if (vel_left == 0)
    {
        PWM_left_value = 0;
    }

    if (vel_right == 0)
    {
        PWM_right_value = 0;
    }

    if (PWM_left_value > 255)
    {
        PWM_left_value = 255;
    }

    if (PWM_right_value > 255)
    {
        PWM_right_value = 255;
    }

    if (PWM_left_value < -255)
    {
        PWM_left_value = -255;
    }

    if (PWM_right_value < -255)
    {
        PWM_right_value = -255;
    }

    pwm_left_msg.data = PWM_left_value;
    pwm_right_msg.data = PWM_right_value;

    PWM_LEFT.publish(&pwm_left_msg);
    PWM_RIGHT.publish(&pwm_right_msg);
}

void agv_run(char DIR1, char DIR2, int PWM_left_value, int PWM_right_value)
{
    digitalWrite(DIR_1, DIR1);
    analogWrite(PWM_1, abs(PWM_left_value));

    digitalWrite(DIR_2, DIR2);
    analogWrite(PWM_2, abs(PWM_right_value));
}

void imu()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    imu_msg.data = g.gyro.z;
    IMU.publish(&imu_msg);
}

void imu_df()
{
    if (IMU_sensor.available())
    {
        if (IMU_sensor.Gyro.Z > 250)
        {
            imu_msg.data = (500 - IMU_sensor.Gyro.Z) * -0.017453;
            IMU.publish(&imu_msg);
        }
        else
        {
            imu_msg.data = 0.017453 * IMU_sensor.Gyro.Z;
            IMU.publish(&imu_msg);
        }
    }
}

void stop_move()
{

    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
}

void srv_clear_data(const Test::Request &req, Test::Response &res)
{
    enc_right_msg.data = 0;
    enc_left_msg.data = 0;
    count_l = 0;
    count_r = 0;
    res.output = "Data has been cleared ";
}

void pid_clear()
{
    prv_error_left = 0;
    prv_error_right = 0;

    error_left = 0;
    error_right = 0;

    I_error_left = 0;
    I_error_right = 0;

    D_error_left = 0;
    D_error_right = 0;
}

void countL()
{
    if (digitalRead(E2B) == 0)
    {
        if (digitalRead(E2A) == 0)
        {
            enc_left_msg.data = count_l++;
//            enc_left_msg.data = count_l++;
        }
        else
        {
            enc_left_msg.data = count_l--;
//            enc_left_msg.data = count_l--;
        }
    }
}

void countR()
{
    if (digitalRead(E1B) == 0)
    {
        if (digitalRead(E1A) == 0)
        {
            enc_right_msg.data = count_r--;
//            enc_right_msg.data = count_r--;
        }
        else
        {
            enc_right_msg.data = count_r++;
//            enc_right_msg.data = count_r++;
        }
    }
}
