#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3, Twist
import pyproj
import math

class GpsToOdomConverter:
    def __init__(self):
        rospy.init_node('gps_to_odom_converter')
        
        # Initialize subscribers and publishers
        self.gps_sub = rospy.Subscriber('/mtk_gps/fix', NavSatFix, self.gps_callback)
        self.odom_pub = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        
        # Initialize projection (UTM transformation)
        self.projector = pyproj.Proj(proj='utm', zone=30, ellps='WGS84')  # Adjust zone based on your location
        
        # Store initial position
        self.init_lon = None
        self.init_lat = None
        
        # Set frame IDs
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        
    def gps_callback(self, msg):
        if not msg.latitude or not msg.longitude:
            return
            
        # Store initial position
        if self.init_lon is None or self.init_lat is None:
            self.init_lon = msg.longitude
            self.init_lat = msg.latitude
            
        # Convert to UTM
        east, north = self.projector(msg.longitude, msg.latitude)
        init_east, init_north = self.projector(self.init_lon, self.init_lat)
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Set position relative to initial position
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = east - init_east
        odom.pose.pose.position.y = north - init_north
        odom.pose.pose.position.z = msg.altitude if msg.altitude else 0.0
        
        # Set orientation (identity quaternion as we don't have orientation from GPS)
        odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        # Set covariance
        # GPS horizontal accuracy is typically 2-3 meters
        odom.pose.covariance[0] = 2.0  # x
        odom.pose.covariance[7] = 2.0  # y
        odom.pose.covariance[14] = 4.0  # z
        odom.pose.covariance[21] = 99999.0  # rotation x
        odom.pose.covariance[28] = 99999.0  # rotation y
        odom.pose.covariance[35] = 99999.0  # rotation z
        
        # Set velocity (we don't have this from GPS)
        odom.twist.twist = Twist()
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        converter = GpsToOdomConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
