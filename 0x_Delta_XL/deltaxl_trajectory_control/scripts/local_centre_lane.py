#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import math
import tf
import tf2_ros
import numpy as np


class LaserScanToGlobalPoints:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('laser_scan_to_points', anonymous=True)

        # Subscribe to the /scan topic
        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)

        # Subscribe to the /odom topic to get the robot's global pose
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # Publisher for the centroid marker
        self.marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)

        # Publisher for the array of markers representing the clipped points
        self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'front_laser', rospy.Time(0), rospy.Duration(1.0))

        # Extract translation components (x, y)
        self.dist_laser_x = transform.transform.translation.x
        self.dist_laser_y = transform.transform.translation.y
        print(self.dist_laser_x, self.dist_laser_y)

        # Initialize the robot's pose in the /odom frame
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.centroid_x = 0.0
        self.centroid_y = 0.0

        self.centre_left_x = 0.0
        self.centre_left_y = 0.0
        self.centre_right_x = 0.0
        self.centre_right_y = 0.0

    def odom_callback(self, msg):
        # Extract the robot's position and orientation from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract orientation in quaternion and convert it to Euler angles (roll, pitch, yaw)
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        

    def scan_callback(self, scan):
        # Convert LaserScan ranges into (x, y) points in local coordinates
        local_points = []

        # Iterate through each range reading
        for i, distance in enumerate(scan.ranges):
            # Ignore invalid or out-of-range data
            if math.isinf(distance) or math.isnan(distance) or distance < scan.range_min or distance > scan.range_max:
                continue

            # Calculate the angle of the current range reading
            angle = scan.angle_min + i * scan.angle_increment

            # Convert polar coordinates (distance, angle) to Cartesian coordinates (x, y) in the robot's local frame
            x_local = distance * math.cos(angle)
            y_local = distance * math.sin(angle)

            # Add the point to the list
            local_points.append((x_local, y_local))

        # Transform local points to global coordinates
        global_points = self.transform_to_global(local_points)

        # Clip the points within a certain rectangle and publish markers
        clipped_points = self.clip_rectangle(global_points)
        cmd_vel_2 = Twist()
        cmd_vel_3 = Twist()
        # Calculate and publish the centroid if there are enough points
        self.centroid_x, self.centroid_y = self.centroid(clipped_points)
        positive_quadrant = self.centroid_x - self.robot_x
            
        if clipped_points:
            self.publish_centroid_marker(self.centroid_x, self.centroid_y)
            distance_to_goal = (self.centroid_x - self.robot_x)**2 + (self.centroid_y - self.robot_y)**2            
            if distance_to_goal > 0.01 and positive_quadrant>0:
                self.control_to_waypoint()

        if not clipped_points or positive_quadrant<0: 
            if abs(positive_quadrant) < 0.5:
                cmd_vel_2.linear.x = 0.15
                cmd_vel_2.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_2)
            else:
                cmd_vel_3.linear.x = 0.15
                cmd_vel_3.angular.z = 0.3
                self.cmd_vel_pub.publish(cmd_vel_3)



    def transform_to_global(self, local_points):
        # Transform local (x, y) coordinates to global /odom coordinates
        global_points = []
        for (x_local, y_local) in local_points:
            # Apply rotation and translation to transform to global coordinates
            x_global = self.robot_x + (x_local * math.cos(self.robot_yaw) - y_local * math.sin(self.robot_yaw))
            y_global = self.robot_y + (x_local * math.sin(self.robot_yaw) + y_local * math.cos(self.robot_yaw))

            global_points.append((x_global, y_global))
        return global_points


    def centroid(self, points):
        # Calculate the centroid of the points
        # we will create two clusters one for the left lane and other for right lane and then take the centroid 
        left_points = []
        right_points =[]
        for point in points:
            if point[1]<0:
                left_points.append(point)
            else:
                right_points.append(point)
        
        if len(left_points)>0 and len(right_points)>0 : 
            self.centre_left_x = sum([point[0] for point in left_points]) / len(left_points)
            self.centre_right_x = sum([point[0] for point in right_points]) / len(right_points)  

            self.centre_left_y = sum([point[1] for point in left_points]) / len(left_points)
            self.centre_right_y = sum([point[1] for point in right_points]) / len(right_points)  
        centre_x = (self.centre_left_x + self.centre_right_x)/2
        centre_y = (self.centre_left_y + self.centre_right_y)/2
        return centre_x, centre_y

    def clip_rectangle(self, points):
        # Clip points within a specified rectangular region and create markers
        clipped_points = []
        marker_array = MarkerArray()
        new_point_x = 0
        new_point_y = 0 
        for idx, point in enumerate(points):
            if (point[1] - self.robot_y) < 1.3 and (point[1] - self.robot_y) > -1.3 and (point[0] - self.robot_x) < 1.5 and (point[0]- self.robot_x)> 0.2:
                new_point_x = point[0] + self.dist_laser_x
                new_point_y = point[1] + self.dist_laser_y
                clipped_points.append((new_point_x, new_point_y))

                # Create a cube marker for each clipped point
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "clipped_area"
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = new_point_x
                marker.pose.position.y = new_point_y
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05  # Set the size of each cube marker
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0  # Set transparency
                marker.color.r = 0.0  # Blue color
                marker.color.g = 0.0
                marker.color.b = 1.0

                # Add the marker to the array
                marker_array.markers.append(marker)

        # Publish the array of markers
        self.clipped_marker_pub.publish(marker_array)
        return clipped_points

    def publish_centroid_marker(self, x, y):
        # Create and publish a marker for the centroid
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "centroid"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15  # Set the size of the centroid marker
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0  # Set transparency
        marker.color.r = 0.0  # Red color
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.marker_pub.publish(marker)
        
    def control_to_waypoint(self):
        # Calculate the difference between current position and waypoint
        delta_x = self.centroid_x - self.robot_x
        delta_y = self.centroid_y - self.robot_y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        target_angle = math.atan2(delta_y, delta_x)
        angle_diff = target_angle - self.robot_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        cmd_vel = Twist()

        # If the distance is greater than the threshold, move towards the waypoint
        if distance > 0.05:
            cmd_vel.linear.x = 0.15
            cmd_vel.angular.z = 0.6 * angle_diff
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        # Keep the node running
        rospy.spin()


if __name__ == '__main__':
    try:
        # Create the node and run it
        node = LaserScanToGlobalPoints()
        node.run()
    except rospy.ROSInterruptException:
        pass
