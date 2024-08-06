#include <tf/tf.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sensors/imu_data.h"

sensors::imu_data euler_msg;

void simulationImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  tf::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(euler_msg.orientation.x, euler_msg.orientation.y, euler_msg.orientation.z);

  if(euler_msg.orientation.x < 0)
    euler_msg.orientation.x = euler_msg.orientation.x + M_PI * 2;
  
  if(euler_msg.orientation.y < 0)
    euler_msg.orientation.y = euler_msg.orientation.y + M_PI * 2;

  if(euler_msg.orientation.z < 0)
    euler_msg.orientation.z = euler_msg.orientation.z + M_PI * 2;

  euler_msg.orientation.x = euler_msg.orientation.x * 180 / M_PI;
  euler_msg.orientation.y = euler_msg.orientation.y *180 / M_PI;
  euler_msg.orientation.z = euler_msg.orientation.z *180 / M_PI;
  euler_msg.acceleration.x = msg->linear_acceleration.x;
  euler_msg.acceleration.y = msg->linear_acceleration.y;
  euler_msg.acceleration.z = msg->linear_acceleration.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_conversion_node");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("/imu_data/quat", 1, simulationImuCallback);
  ros::Publisher imu_pub = n.advertise<sensors::imu_data>("imu_data", 1);

  ros::Rate rate(60);
  
  rate.sleep();
  ros::spinOnce();

  while (ros::ok())
  {
    imu_pub.publish(euler_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
