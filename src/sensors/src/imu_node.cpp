#include <iostream>
#include <strings.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <ros/ros.h>
#include "sensors/imu_data.h"

std::vector<float> split (std::string str, char delimeter)
{
    std::vector<float> vals;
    std::string num = "";
    for(int i = 0; i < str.length(); i++)
    {
        if(str[i] == delimeter || i == str.length() - 1)
        {
            try{
                vals.push_back(std::stof(num));
                
            }catch (...){

            }
            num = "";
        }   
        else 
            num = num + str[i];
    }
    
    return vals;
}
int main()
{

    // ROS Initiation

    ros::init(argc,argv,"imu_node");

    pub = n.advertise<sensors::imu_data>("imu",10);

    // Defining variables

    char data;
    int n;
    std::string imu_data = "";
    std::vector<float> imu;
    sensors::imu_data values;

    // Defining IMU port and opening 
    
    int port = open("/dev/nano", O_RDWR | O_NOCTTY | O_SYNC);
    if (port < 0)
        std::cout << ("Error") << std::endl;
    while (1)
    {
        n = read(port, &data, sizeof(data));
        if(data == '\n')
        {
            do{
                n = read(port, &data, sizeof(data));
                imu_data = imu_data + data;
            }while(data != '\n');
            imu = split(imu_data, ' ');

            for(int i = 0; i < imu.size(); i++)
            {
                values.acceleration.x = imu[0];
                values.acceleration.y = imu[1];
                values.acceleration.z = imu[2];

                values.orientation.x = imu[3];
                values.orientation.x = imu[4];
                values.orientation.x = imu[5];

                pub.publish();
            }
            std::cout << std::endl;
            imu_data = "";
        }
        ros::spin();
    } 
    close(port);
}
