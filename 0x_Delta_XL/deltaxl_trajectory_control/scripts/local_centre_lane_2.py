#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import geometry_msgs
import math
import tf
import tf2_ros
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray

class LaserScanToGlobalPoints:
    def __init__(self):
        rospy.init_node('laser_scan_to_points', anonymous=True)
        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)
        # Publisher for the centroid marker
        self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)
        self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'front_laser', rospy.Time(0), rospy.Duration(1.0))
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
        
        # Calculate and publish the centroid if there are enough points
        if clipped_points:
            left_centers, right_centers = self.centroid(clipped_points)
            self.centroid_x, self.centroid_y = self.centre_of_line(left_centers, right_centers)
        distance_to_goal = (self.centroid_x - self.robot_x)**2 + (self.centroid_y - self.robot_y)**2
        if distance_to_goal > 0.01:
            self.control_to_waypoint()

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
        # Separate points into left and right quadrants
        left_points = np.array([point for point in points if point[1] < 0])
        right_points = np.array([point for point in points if point[1] > 0])

        # Initialize lists to store cluster centers
        left_centers = []
        right_centers = []

        # Cluster the points in the left quadrant and find cluster centers
        if len(left_points) > 0:
            left_clusters = self.cluster_points(left_points)
            for cluster in left_clusters:
                left_centers.append(np.mean(cluster, axis=0))

        # Cluster the points in the right quadrant and find cluster centers
        if len(right_points) > 0:
            right_clusters = self.cluster_points(right_points)
            for cluster in right_clusters:
                right_centers.append(np.mean(cluster, axis=0))

        # Draw lines between the cluster centers in each quadrant
        self.publish_lines(left_centers, right_centers)
        return left_centers, right_centers

    def cluster_points(self, points):
        # Use DBSCAN clustering algorithm to find clusters
        db = DBSCAN(eps=0.3, min_samples=3).fit(points)
        labels = db.labels_

        # Group points by cluster
        clusters = []
        for label in set(labels):
            if label != -1:  # Ignore noise points labeled as -1
                cluster = points[labels == label]
                clusters.append(cluster)

        return clusters

    # Adjust the method to publish a MarkerArray
    def publish_lines(self, left_centers, right_centers):
        # Create a MarkerArray to publish the lines
        line_array = MarkerArray()

        # Create lines for the left quadrant
        for i, center in enumerate(left_centers[:-1]):
            next_center = left_centers[i + 1]
            line_array.markers.append(self.create_line_marker(center, next_center, i, "left_lines"))

        # Create lines for the right quadrant
        for i, center in enumerate(right_centers[:-1]):
            next_center = right_centers[i + 1]
            line_array.markers.append(self.create_line_marker(center, next_center, i + 100, "right_lines"))

        # Publish the array of lines
        self.marker_pub.publish(line_array)

    def create_line_marker(self, start, end, marker_id, ns):
        # Create a line marker between two points
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Set transparency
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Add the start and end points to the line
        marker.points.append(self.create_point(start))
        marker.points.append(self.create_point(end))

        return marker

    def create_point(self, coords):
        # Create a Point object from (x, y) coordinates
        point = geometry_msgs.msg.Point()
        point.x = coords[0]
        point.y = coords[1]
        point.z = 0
        return point
    
    def clip_rectangle(self, points):
        # Clip points within a specified rectangular region and create markers
        clipped_points = []
        marker_array = MarkerArray()
        new_point_x = 0
        new_point_y = 0 
        for idx, point in enumerate(points):
            if (point[1] - self.robot_y) < 1.3 and (point[1] - self.robot_y) > -1.3 and (point[0] - self.robot_x) < 4.5 and (point[0]- self.robot_x)> 0.2:
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
    
    def centre_of_line(self,left_centers, right_centers):
        center_left_quadrant = (left_centers[0]+left_centers[-1])/2
        center_right_quadrant = (right_centers[0]+right_centers[-1])/2
        waypoint = (center_left_quadrant + center_right_quadrant)/2
        self.publish_centroid_marker(waypoint[0], waypoint[1])
        return waypoint[0], waypoint[1]


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
        self.centroid_marker_pub.publish(marker)

    def control_to_waypoint(self):
        # Calculate the difference between current position and waypoint
        delta_x = self.centroid_x - self.robot_x
        delta_y = self.centroid_y - self.robot_y

        # Calculate the distance to the waypoint
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Calculate the angle to the waypoint
        target_angle = math.atan2(delta_y, delta_x)

        # Calculate the difference between current orientation and target angle
        angle_diff = target_angle - self.robot_yaw

        # Normalize angle_diff to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Initialize Twist message
        cmd_vel = Twist()

        # If the distance is greater than the threshold, move towards the waypoint
        if distance > 0.05:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.6 * angle_diff
        else:
            # Stop if within threshold distance to the waypoint
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # Publish the velocity command
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
