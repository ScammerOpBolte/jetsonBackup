#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <thread>

#include <limits> // for std::numeric_limits

std::atomic<int> state(1);

void publishState(ros::Publisher& pub) {
    std_msgs::Int32 msg;
    ros::Rate rate(3); // Adjust as needed
    
    while (ros::ok()) {
        // Publish the current state value
        msg.data = state.load();
        pub.publish(msg);
        ROS_INFO("Published state: %d", msg.data);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher_cpp");
    ros::NodeHandle nh;

    ros::Publisher state_pub = nh.advertise<std_msgs::Int32>("state_topic", 10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::thread publisher_thread(publishState, std::ref(state_pub));

    std::string input;
    while (ros::ok()) {
        std::cout << "Enter a new state value (0, 1, 2) or press 'q' to quit: ";
        std::cin >> input;

        if (input == "q") {
            break; // Exit the loop if 'q' is entered
        }

        int new_state;
        try {
            new_state = std::stoi(input);
            if (new_state >= 0 && new_state <= 2) {
                state.store(new_state);
            } else {
                std::cerr << "Invalid state value! Please enter a value between 0 and 2." << std::endl;
            }
        } catch(const std::invalid_argument& e) {
            std::cerr << "Invalid input! Please enter a valid integer." << std::endl;
        } catch(const std::out_of_range& e) {
            std::cerr << "Input out of range! Please enter a value within the integer range." << std::endl;
        }
        // Clear the input buffer
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    // Wait for the publisher thread to finish
    publisher_thread.join();
    spinner.stop();

    return 0;
}