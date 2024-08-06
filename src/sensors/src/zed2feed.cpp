#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> 

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");

    //Inintializing topics
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_full = it.advertise("zed2/image", 5);
    image_transport::Publisher pub_left = it.advertise("zed2/left/image", 5);
    image_transport::Publisher pub_right = it.advertise("zed2/right/image", 5);

    //Receiving camera feed
    cv::VideoCapture cap(0);
    //cv::VideoCapture cap("/dev/Zed2");
    if(!cap.isOpened()) return 1;
    cv::Mat frame_full,frame_left,frame_right;

    sensor_msgs::ImagePtr msg_full;
    sensor_msgs::ImagePtr msg_left;
    sensor_msgs::ImagePtr msg_right;
    ros::Rate rate(5);
    while (nh.ok()) {
        cap >> frame_full;
        if(!frame_full.empty()) {
            msg_full = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_full).toImageMsg();
            frame_left = frame_full(Rect(0,0,672,376)).clone();
            frame_right = frame_full(Rect(672,0,672,376)).clone();
            msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_left).toImageMsg();
            msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_right).toImageMsg();
            pub_full.publish(msg_full);
            pub_left.publish(msg_left);
            pub_right.publish(msg_right);
            cv::waitKey(1);
        }
        ros::spinOnce();
        rate.sleep();
  }
}
