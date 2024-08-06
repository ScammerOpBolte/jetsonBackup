#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import csv
from rospy import Duration


class GPSLogger:
    def __init__(self):
        rospy.init_node('gps_logger_node', anonymous=True)

        # Define the GPS topic you want to subscribe to
        self.gps_topic = "/fix"  # Change this to your GPS topic

        # Initialize a list to store GPS coordinates
        self.gps_data = []

        # Subscribe to the GPS topic
        rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)
        # self.rate = rospy.Rate(5)  # 1 Hz


        # Create a timer to trigger GPS data collection every 5 seconds
        self.timer = rospy.Timer(Duration(1), self.save_gps_data)

    def gps_callback(self, msg):
        # Callback function to handle incoming GPS data
        latitude = msg.latitude
        longitude = msg.longitude

        # Append GPS data to the list
        self.gps_data.append((latitude, longitude))

    def save_gps_data(self, event):
        # Save GPS data to a CSV file at each timer event
        self.save_to_csv('gps_data.csv')

    def save_to_csv(self, filename):
        # Save GPS data to a CSV file
        with open(filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Latitude', 'Longitude'])
            csv_writer.writerows(self.gps_data)
            
    # def normalize_data(self, data):
    #     # Normalize data using MinMaxScaler
    #     scaler = MinMaxScaler()
    #     data_normalized = scaler.fit_transform([[x] for x in data])
    #     return [val[0] for val in data_normalized]
    

    def run(self):
        # Run the ROS node
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the GPSLogger class
    gps_logger = GPSLogger()

    # Run the ROS node and log GPS data
    
    gps_logger.run()
    # finally:
    #     # Plot the GPS data using Matplotlib
    #     gps_logger.plot_gps_data()