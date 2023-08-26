#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray  # For publishing multiple float values in one message
import math
from utility.constants import GPS_LONG, GPS_LAT, GPS_HEADING

class GPSTracking:
    def __init__(self):
        # Initialize node
        rospy.init_node('gps_tracking_node', anonymous=True)

        # Attributes
        self.current_lat = None
        self.current_long = None
        self.current_heading = 0.0
        self.gps_recieved = False
        self.compass_recieved = False
        
        # Subscribers
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.compass_callback)

        # Publisher
        self.result_pub = rospy.Publisher('/controller/waypoint_position', Float64MultiArray, queue_size=10)

        

    def gps_callback(self, data):
        self.current_lat = data.latitude
        self.current_long = data.longitude
        self.gps_recieved = True
        self.calculate_and_publish()

    def compass_callback(self, data):
        self.current_heading = data.data
        self.compass_recieved = True

    def calculate_and_publish(self):
        print("RUNNING")
        if (self.compass_recieved and self.gps_recieved):
            # Distance calculation (Using Haversine formula)
            R = 6371000  # Earth radius in meters
            phi1 = math.radians(self.current_lat)
            phi2 = math.radians(GPS_LAT)
            delta_phi = math.radians(GPS_LAT - self.current_lat)
            delta_lambda = math.radians(GPS_LONG - self.current_long)
            
            a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
            c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
            
            distance = R * c

            # Angle calculation (simple difference)
            change_in_angle = GPS_HEADING - self.current_heading

            # Normalize the angle to [-180, 180]
            while change_in_angle > 180:
                change_in_angle -= 360
            while change_in_angle < -180:
                change_in_angle += 360

            # Publish the results
            result_msg = Float64MultiArray()
            result_msg.data = [-change_in_angle, distance]
            print("Change in Angle: ", -change_in_angle, "|| Distance to Point", distance)
            self.result_pub.publish(result_msg)

if __name__ == '__main__':
    try:
        node = GPSTracking()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
