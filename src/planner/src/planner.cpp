#include "include/planner.hpp"

int main (int argc, char** argv)
{

    // ROS Node initialization 

    ros::init (argc, argv, "planner_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Setting ROS Topics and Rate 

    std::string imu_topic = "/imu_data";
    std::string gps_topic = "/fix";
    std::string aruco_topic = "/aruco_detect";
    std::string lidar_topic = "/scan";
    // std::string aruco_yolo_topic = "/aruco_yolo_detect";

    // Creating Object for Global Planning 

    planner::SensorCallback control_class (imu_topic, aruco_topic, gps_topic, lidar_topic);

    // Waiting for node to be killed

    ros::waitForShutdown();
}