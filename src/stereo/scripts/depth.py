import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import pyrealsense2 as rs

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# Initialize ROS node
rospy.init_node('realsense_node')

# Create publishers for the different image streams
depth_pub = rospy.Publisher('/realsense/depth', Image, queue_size=1)
color_pub = rospy.Publisher('/realsense/color', Image, queue_size=1)
depth_highlighted_red_pub = rospy.Publisher('/realsense/depth_highlighted_red', Image, queue_size=1)
depth_heat_map_pub = rospy.Publisher('/realsense/depth_heat_map', Image, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Create CvBridge
bridge = CvBridge()

val = 800
try:
    while not rospy.is_shutdown():
        # Wait for a coherent pair of frames: depth and color
        twist_msg = Twist()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Create a three-channel image with the same size as depth_image
        depth_highlighted_red = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
        # Set the red channel to 255 for highlighted regions
        depth_highlighted_red[depth_image > val, 2] = 255

        # Count the number of red pixels
        red_pixel_count = np.count_nonzero(depth_image > val)

        # Calculate the percentage of red pixels
        red_pixel_percentage = (red_pixel_count / (depth_image.shape[0] * depth_image.shape[1])) * 100

        # Print the percentage of red pixels
        print(f"Percentage of red pixels: {red_pixel_percentage}%")
        twist_msg.linear.x = 0.0  # Adjust linear velocity as needed
        twist_msg.angular.z = 0.0  # Adjust angular velocity as needed
        cmd_vel_pub.publish(twist_msg)

        # Check if more than 10% of the depth map is red
        if red_pixel_percentage > 20:
            # Publish to /cmd_vel (you may need to adjust the values)
            twist_msg.linear.x = -0.0  # Adjust linear velocity as needed
            twist_msg.angular.z = 0.0  # Adjust angular velocity as needed
            cmd_vel_pub.publish(twist_msg)

        # Convert images to ROS Image messages
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_highlighted_red_msg = bridge.cv2_to_imgmsg(depth_highlighted_red, encoding="bgr8")

        # Create a heat map using the jet color map
        heat_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
        heat_map_msg = bridge.cv2_to_imgmsg(heat_map, encoding="bgr8")

        # Publish images
        depth_pub.publish(depth_msg)
        color_pub.publish(color_msg)
        depth_highlighted_red_pub.publish(depth_highlighted_red_msg)
        depth_heat_map_pub.publish(heat_map_msg)

except rospy.ROSInterruptException:
    pass
finally:
    # Stop streaming
    pipeline.stop()
