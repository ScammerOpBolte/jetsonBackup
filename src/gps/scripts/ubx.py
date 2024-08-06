import rospy
from sensor_msgs.msg import NavSatFix
from serial import Serial
from pyubx2 import UBXReader
import sys
import socket
import threading
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 12360
s.connect(('10.0.0.101', port))

class UbxParser:
    def __init__(self):
        self.gps_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
        self.stream = Serial('/dev/ttyTHS0', 38400)

    def sending_rtcm(self):
        while True:
            received_rtcm = s.recv(1024)
            if not received_rtcm:
                break
            # print(received_rtcm)
            print("receiving rtcm")
            self.stream.write(received_rtcm)

    def receiver_gps(self):
        while True:
            ubr = UBXReader(self.stream)
            try:
                (raw_data, parsed_data) = ubr.read()
                if parsed_data.identity == "NAV-PVT":
                    fixType,lat, lon, alt = parsed_data.fixType,parsed_data.lat, parsed_data.lon, parsed_data.hMSL
                    print(f"FixType={fixType},lat = {lat}, lon = {lon}, alt = {alt/1000} m")
                    msg = NavSatFix()
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = alt
                    self.gps_pub.publish(msg)
            except:
            	continue
    # def run(self):
    #     while not rospy.is_shutdown():
    #         self.sending_rtcm()
    #         self.receiver_gps()

if __name__ == "__main__":
    rospy.init_node('ubx_parser')
    try:
        ubx = UbxParser()
        # ubx.run()
        t1=threading.Thread(target=ubx.sending_rtcm)
        t2=threading.Thread(target=ubx.receiver_gps)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except KeyboardInterrupt:
        sys.exit(0)
