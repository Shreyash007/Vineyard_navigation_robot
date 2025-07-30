#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import geometry_msgs
import math
import tf
import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Int32
  
class LaserScanToGlobalPoints:
    def __init__(self):
        rospy.init_node('point_cloud_to_points', anonymous=True)
        #rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)
        # Publisher for the centroid marker
        self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)
        self.centroid_pub = rospy.Publisher('/centroid', Pose, queue_size=10)
        self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        self.empty_patch_pub = rospy.Publisher('/empty_patch', Int32, queue_size=10)

        # Variables to hold camera model and PointCloud data
        self.cam_model = PinholeCameraModel()
        self.point_cloud = None
        
        # Set the width and height of the image (replace with actual camera resolution)
        self.width = 640  
        self.height = 480 
        
        # Subscriber to the camera info for camera model
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # Subscriber to the point cloud topic
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        
        # Publisher for filtered point cloud data
        self.pub = rospy.Publisher('/filtered/points', PointCloud2, queue_size=10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'camera_depth_frame', rospy.Time(0), rospy.Duration(1.0))
        self.dist_camera_x = transform.transform.translation.x
        self.dist_camera_y = transform.transform.translation.y
        self.dist_camera_z = transform.transform.translation.z
        #print(self.dist_laser_x, self.dist_laser_y)

        # Initialize the robot's pose in the /odom frame
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0

        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_z = 0.0

        self.centre_left_x = 0.0
        self.centre_left_y = 0.0
        self.centre_right_x = 0.0
        self.centre_right_y = 0.0
        self.empty_patch = 0

        self.x_bound_box = 0.2
        self.y_bound_box = 1.5
        self.z_bound_box = 5


    def odom_callback(self, msg):
        # Extract the robot's position and orientation from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

        # Extract orientation in quaternion and convert it to Euler angles (roll, pitch, yaw)
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        position = Pose()
        position.position.x = self.centroid_x
        position.position.y = self.centroid_y
        position.position.z = self.centroid_z
        
        self.centroid_pub.publish(position)

    def camera_info_callback(self, cam_info):
        # Set the camera model once we get the camera info
        self.cam_model.fromCameraInfo(cam_info)

    def downsample_point_cloud(self, points, downsample_factor):
        """Randomly downsample the point cloud data."""
        if len(points) > downsample_factor:
            indices = np.random.choice(len(points), size=len(points) // downsample_factor, replace=False)
            return points[indices]
        return points

    def pointcloud_callback(self, data):
        # Store the point cloud data
        self.point_cloud = point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)
        
        # Downsample the point cloud
        downsample_factor = 20  # Adjust this based on your requirements
        downsampled_points = self.downsample_point_cloud(np.array(self.point_cloud), downsample_factor)
        
        #change in axes to represent the camera frame and odom frame 
        filtered_points = np.array([[p[2], -p[0], p[1]] for p in downsampled_points if -self.x_bound_box <= p[1] <= self.x_bound_box and -self.y_bound_box <= p[0] <= self.y_bound_box and p[2] <= self.z_bound_box])
        global_points = self.transform_to_global(filtered_points)
        left_centers, right_centers = self.centroid(global_points)
        clipped_points, self.empty_patch = self.clip_rectangle(global_points)
        if len(left_centers)>0 and len(right_centers)>0 and self.empty_patch == 0:
            self.centroid_x, self.centroid_y, self.centroid_z = self.centre_of_line(left_centers, right_centers)
        self.empty_patch_pub.publish(self.empty_patch)
        

    def transform_to_global(self, local_points):
        # Transform local (x, y) coordinates to global /odom coordinates
        global_points = []
        for (x_local, y_local, z_local) in local_points:
            # Apply rotation and translation to transform to global coordinates
            x_global = self.robot_x + (x_local * math.cos(self.robot_yaw) - y_local * math.sin(self.robot_yaw)) + self.dist_camera_x
            y_global = self.robot_y + (x_local * math.sin(self.robot_yaw) + y_local * math.cos(self.robot_yaw)) + self.dist_camera_y
            z_global = self.robot_z + z_local + self.dist_camera_z

            global_points.append((x_global, y_global, z_global))
        return global_points


    def centroid(self, points):
        # Separate points into left and right quadrants
        left_points = np.array([point for point in points if (point[1] - self.robot_y)< 0])
        right_points = np.array([point for point in points if (point[1] - self.robot_y) > 0])

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
        db = DBSCAN(eps=0.3, min_samples=20).fit(points)
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
        
        for idx, point in enumerate(points):
                clipped_points.append(point)
                # Create a cube marker for each clipped point
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "clipped_area"
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.03  # Set the size of each cube marker
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.a = 1.0  # Set transparency
                marker.color.r = 0.0  # Blue color
                marker.color.g = 0.0
                marker.color.b = 1.0

                # Add the marker to the array
                marker_array.markers.append(marker)
        
        if not clipped_points:
            self.empty_patch = 1
            print('empty_patch:',self.empty_patch)
        else:
            self.empty_patch = 0
            print('empty_patch:',self.empty_patch)
        # Publish the array of markers
        self.clipped_marker_pub.publish(marker_array)
        return clipped_points, self.empty_patch
    
    def centre_of_line(self,left_centers, right_centers):
        if len(left_centers)>0 and len(right_centers)>0:
            center_left_quadrant = (left_centers[0]+left_centers[-1])/2
            center_right_quadrant = (right_centers[0]+right_centers[-1])/2
            self.waypoint = (center_left_quadrant + center_right_quadrant)/2
        # self.publish_centroid_marker(waypoint[2], waypoint[1])
        self.publish_centroid_marker(self.waypoint[0], self.waypoint[1])
        return self.waypoint[0], self.waypoint[1], self.waypoint[2]


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
