#include <tf/tf.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

const double kRoverHeight = 0.37;
double groundDis = kRoverHeight/cos((M_PI/180)*75);
// double ditchDistance = cos(15*(M_PI/180))*groundDis; //1.380859
double ditchDistance = 1.8;
const double kditchDepth = 0.47;
const double kRoverLength = 0.5;
const double kRoverBreadth = 0.4;
const double kLidarPos = 0.02; 
const int kLidarAngularRange = 270;
const int kLidarSamples = 1101;

int lidar_center = kLidarSamples / 2;
// int lidar_min_range = lidar_center - ((atan(kRoverBreadth / 2 / kLidarPos) * 180 / M_PI) * kLidarSamples / kLidarAngularRange);
// int lidar_max_range = lidar_center + ((atan(kRoverBreadth / 2 / kLidarPos) * 180 / M_PI) * kLidarSamples / kLidarAngularRange);

std::vector<float> lidar_samples;

typedef struct{
    double roll;
    double pitch;
    double yaw;
}angles;

angles ang;

int ditchDetect()
{
    // ROS_INFO("%f",ditchDistance);
    float depth;
    int count=0;
    for (int i = lidar_center-10; i <= lidar_center+10; i++)
    {
        if (lidar_samples[i]>ditchDistance)
        {
            depth=cos((M_PI/180)*75)*(lidar_samples[i]-groundDis);
            ROS_INFO("Distance = %f Depth = %f",lidar_samples[i],depth);

            if(depth>kditchDepth)
            {
                count++;
            }
        }
    }
    if (count>=21)
        return 1;
    return 0;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &las)
{
    lidar_samples = las->ranges;
    ROS_INFO("Ditch = %d",ditchDetect());
};

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(ang.roll,ang.pitch, ang.yaw); 
}

int main (int argc, char** argv)
{
    ros::init(argc,argv,"ditch_detect");
    ros::NodeHandle n;
    ros::Subscriber lidar_below_sub = n.subscribe("/scan_250", 1, lidarCallback);
    ros::Subscriber imu_sub = n.subscribe("/imu", 1, imuCallback);
    ros::spin();
}