
// ##################################################################//
//  Ros version : Noetic
//  Drive type: Mechanum 4wd
//  Date : 12/6/2023
//  Rev. : 1
//  Note : Calculate in MCU
// ##################################################################//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
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

// Encoder
long int enc_count[4];
long int enc_count_0;
long int enc_count_1;
long int enc_count_2;
long int enc_count_3;
long int prv_enc_count[4];

// CMD vel
float vel_x, vel_y, vel_z;
int DIR_[4]; // motor direction
float vel[4];
float vel_act[4];
float vel_flt[4];
float vel_prv[4];
float d_time;
float distance[4];
unsigned long prv_time[2];
const int TPM[] = {902,1003}; // tick/meter
const int pub_rate = 50;        // Hz
float lx = 0.3;                 // m
float ly = 0.3;                 // m
float wheel_diameter = 0.127;   // m

// LED
const uint8_t LED[] = {19, 20, 21}; // R,G,B

// PID
float prv_err[4];
float err[4];
float I_err[4];
float D_err[4];
float KP = 50;
float KI = 10;
float KD = 5;

// Motor
int pwm_value[4];
int base_width = 0.450;

// IR sensor
const uint8_t IR[] = {14, 15};

// ##################################################################//
ros::NodeHandle nh;
// Pub IMU data
std_msgs::Float32 imu_msg;
ros::Publisher IMU("imu_value", &imu_msg);

// Pub Encoder data
std_msgs::Int32 enc_msg_0;
ros::Publisher ENC_0("enc_value_0", &enc_msg_0);

std_msgs::Int32 enc_msg_1;
ros::Publisher ENC_1("enc_value_1", &enc_msg_1);

std_msgs::Int32 enc_msg_2;
ros::Publisher ENC_2("enc_value_2", &enc_msg_2);

std_msgs::Int32 enc_msg_3;
ros::Publisher ENC_3("enc_value_3", &enc_msg_3);

// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Pub vel data
std_msgs::Float32 vel_msg_0;
ros::Publisher VEL_0("vel_value_0", &vel_msg_0);

std_msgs::Float32 vel_msg_1;
ros::Publisher VEL_1("vel_value_1", &vel_msg_1);

std_msgs::Float32 vel_msg_2;
ros::Publisher VEL_2("vel_value_2", &vel_msg_2);

std_msgs::Float32 vel_msg_3;
ros::Publisher VEL_3("vel_value_3", &vel_msg_3);

// Pub pwm data
std_msgs::Int32 pwm_msg_0;
ros::Publisher PWM_0("pwm_value_0", &pwm_msg_0);

std_msgs::Int32 pwm_msg_1;
ros::Publisher PWM_1("pwm_value_1", &pwm_msg_1);

std_msgs::Int32 pwm_msg_2;
ros::Publisher PWM_2("pwm_value_2", &pwm_msg_2);

std_msgs::Int32 pwm_msg_3;
ros::Publisher PWM_3("pwm_value_3", &pwm_msg_3);

// Pub votage data
std_msgs::Float32 vol_msg;
ros::Publisher VOL("vol_value", &vol_msg);
// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);
// ##################################################################//

void pin_define()
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
        pinMode(IR[i], OUTPUT);
    }
    attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}

void CMD_vel_callback(const geometry_msgs::Twist &CVel)
{
    // w-fl=(vel_x-vel_y-(lx+ly)*vel_z)/r
    // w-fr=(vel_x+vel_y+(lx+ly)*vel_z)/r
    // w-rl=(vel_x+vel_y-(lx+ly)*vel_z)/r
    // w-rr=(vel_x-vel_y+(lx+ly)*vel_z)/r

    vel_x = CVel.linear.x;
    vel_y = CVel.linear.y;
    vel_z = CVel.angular.z;

    if (abs(vel_z) < 0.02)
    {
        vel_z = 0.0;
    }

    if (vel_x == 0.0 && vel_z == 0.0) // stop
    {
        for (int i = 0; i < 4; i++)
        {
            vel[i] = 0.0;
            analogWrite(PWM[i], 0);
            pid_clear();
        }
    }
    else
    {
        vel[0] = (vel_x - vel_y - (lx + ly) * vel_z);
        vel[1] = (vel_x + vel_y + (lx + ly) * vel_z);
        vel[2] = (vel_x + vel_y - (lx + ly) * vel_z);
        vel[3] = (vel_x - vel_y + (lx + ly) * vel_z);
    }
}
ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMD_vel_callback);

void setup()
{
    nh.initNode();
    pin_define();

    // IMU
    Serial.begin(57600);
    mySerial.begin(9600);
    IMU_sensor.modifyFrequency(FREQUENCY_20HZ);

    // pub
    nh.advertise(IMU);

    nh.advertise(ENC_0);
    nh.advertise(ENC_1);
    nh.advertise(ENC_2);
    nh.advertise(ENC_3);

    nh.advertise(VEL_0);
    nh.advertise(VEL_1);
    nh.advertise(VEL_2);
    nh.advertise(VEL_3);

    nh.advertise(PWM_0);
    nh.advertise(PWM_1);
    nh.advertise(PWM_2);
    nh.advertise(PWM_3);

    nh.advertise(VOL);

    // service
    nh.advertiseService(clear_enc_data);

    // sub
    nh.subscribe(sub_cmdVel);

    enc_msg_0.data = 0;
    enc_msg_1.data = 0;
    enc_msg_2.data = 0;
    enc_msg_3.data = 0;
}

void loop()
{
    nh.spinOnce();
}

void *vel_calculate(int enc_count_0, int enc_count_1, int enc_count_2, int enc_count_3)
{
    enc_count[0] = enc_count_0;
    enc_count[1] = enc_count_1;
    enc_count[2] = enc_count_2;
    enc_count[3] = enc_count_3;

    if (millis() - prv_time[0] > 20)
    {
        d_time = millis() - prv_time[0];

        // encoder type1
        for (int i = 0; i < 2; i++)
        {
            distance[i] = float(enc_count[i] - prv_enc_count[i]) / TPM[0];
            vel_act[i] = (distance[i] / d_time) * 1000;
        }

        // encoder type2
        for (int i = 2; i < 4; i++)
        {
            distance[i] = float(enc_count[i] - prv_enc_count[i]) / TPM[1];
            vel_act[i] = (distance[i] / d_time) * 1000;
        }

        // Filter velocity at 25 Hz
        for (int i = 0; i < 4; i++)
        {
            vel_flt[i] = 0.854 * vel_flt[i] + 0.0728 * vel_act[i] + 0.0728 * vel_prv[i];
            vel_prv[i] = vel_flt[i];

            prv_enc_count[i] = float(enc_count[i]);
        }
        prv_time[0] = millis();

        vel_msg_0.data = vel_flt[0];
        vel_msg_1.data = vel_flt[1];
        vel_msg_2.data = vel_flt[2];
        vel_msg_3.data = vel_flt[3];

        VEL_0.publish(&vel_msg_0);
        VEL_1.publish(&vel_msg_1);
        VEL_2.publish(&vel_msg_2);
        VEL_3.publish(&vel_msg_3);

        return vel_flt;
    }
}

void pid(float vel_0, float vel_1, float vel_2, float vel_3)
{
    vel_calculate(enc_count_0, enc_count_1, enc_count_2, enc_count_3);

    vel[0] = vel_0;
    vel[1] = vel_1;
    vel[2] = vel_2;
    vel[3] = vel_3;

    for (int i = 0; i < 4; i++)
    {
        if (vel[i] == 0.0 && err[i] == 0.0)
        {
            I_err[i] = 0;
        }
        if (err[i] > 0.6)
        {
            pid_clear();
        }

        err[i] = vel[i] - vel_flt[i];
        I_err[i] = I_err[i] + err[i];
        D_err[i] = err[i] - prv_err[i];

        pwm_value[i] = KP * err[i] + KI * I_err[i] + KD * D_err[i];
        prv_err[i] = err[i];

        if (vel[i] == 0)
        {
            pwm_value[i] = 0;
        }
        if (pwm_value[i] > 255)
        {
            pwm_value[i] = 255;
        }
        if (pwm_value[i] < -255)
        {
            pwm_value[i] = -255;
        }
    }
    pwm_msg_0.data = pwm_value[0];
    pwm_msg_1.data = pwm_value[1];
    pwm_msg_2.data = pwm_value[2];
    pwm_msg_3.data = pwm_value[3];

    PWM_0.publish(&pwm_msg_0);
    PWM_1.publish(&pwm_msg_1);
    PWM_2.publish(&pwm_msg_2);
    PWM_3.publish(&pwm_msg_3);
}

void agv_run()
{
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

void srv_clear_data(const Test::Request &req, Test::Response &res)
{
    enc_msg_0.data = 0;
    enc_msg_1.data = 0;
    enc_msg_2.data = 0;
    enc_msg_3.data = 0;

    for (int l = 0; l < 4; l++)
    {
        enc_count[l] = 0;
    }
    res.output = "Data has been cleared ";
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

void enc_count0()
{
    if (digitalRead(EB[0]) == 0)
    {
        if (digitalRead(EA[0]) == 0)
        {
            enc_msg_0.data = enc_count_0++;
        }
        else
        {
            enc_msg_0.data = enc_count_0--;
        }
    }
}

void enc_count1()
{
    if (digitalRead(EB[1]) == 0)
    {
        if (digitalRead(EA[1]) == 0)
        {
            enc_msg_1.data = enc_count_1++;
        }
        else
        {
            enc_msg_1.data = enc_count_1--;
        }
    }
}

void enc_count2()
{
    if (digitalRead(EB[2]) == 0)
    {
        if (digitalRead(EA[2]) == 0)
        {
            enc_msg_2.data = enc_count_2++;
        }
        else
        {
            enc_msg_2.data = enc_count_2--;
        }
    }
}

void enc_count3()
{
    if (digitalRead(EB[3]) == 0)
    {
        if (digitalRead(EA[3]) == 0)
        {
            enc_msg_3.data = enc_count_3++;
        }
        else
        {
            enc_msg_3.data = enc_count_3--;
        }
    }
}