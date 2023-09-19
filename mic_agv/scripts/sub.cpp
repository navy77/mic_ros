#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>

void chatterCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data[0].c_str());

  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("enc_data", 1000, chatterCallback);
  ros::spin();
  return 0;
}