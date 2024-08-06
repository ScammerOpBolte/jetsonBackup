import rospy
from sensor_msgs.msg import NavSatFix
from serial import Serial
from pyubx2 import UBXReader
import sys

class UbxParser:
    def __init__(self):
        self.gps_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
        self.stream = Serial('/dev/ttyTHS0', 38400)
    
    def parser(self):
        ubr = UBXReader(self.stream)
        (raw_data, parsed_data) = ubr.read()
        if parsed_data.identity == "NAV-PVT":
            lat, lon, alt = parsed_data.lat, parsed_data.lon, parsed_data.hMSL
            print(f"lat = {lat}, lon = {lon}, alt = {alt/1000} m")
            msg=NavSatFix()
            msg.latitude=lat
            msg.longitude=lon
            msg.altitude=alt
            self.gps_pub.publish(msg)


    def run(self):
        while not rospy.is_shutdown():
            self.parser(self.stream)

if __name__=="__main__":
    rospy.init_node('ubx_parser')
    try:
        ubx=UbxParser()
        ubx.run()
    except KeyboardInterrupt:
        sys.exit(0)
