#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <serial/serial.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_subscriber_cpp");
    ros::NodeHandle n;
    serial::Serial serial_port("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(100));
    ros::Subscriber state_sub = n.subscribe<std_msgs::Int32>("state_topic", 10, [&](const std_msgs::Int32::ConstPtr& msg) {
        int state = msg->data;
        std::string data = std::to_string(state) + "\n";
        std::cout << data << std::endl;
        serial_port.write(data);
    });
    ros::spin();
    serial_port.close();
    return 0;
}