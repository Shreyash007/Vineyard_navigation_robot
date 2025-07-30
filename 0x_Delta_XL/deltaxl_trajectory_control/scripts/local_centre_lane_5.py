#!/usr/bin/env python3
import rospy
import geometry_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Int32
import tf2_ros

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)

        # Subscriber to the point cloud topic
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

        # Publisher for the centroid marker
        self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)
        self.centroid_pub = rospy.Publisher('/centroid', Pose, queue_size=10)
        self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        self.empty_patch_pub = rospy.Publisher('/empty_patch', Int32, queue_size=10)
        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)

        # Set the width and height of the image (replace with actual camera resolution)
        self.width = 640
        self.height = 480

        # Subscriber to the camera info for camera model
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize the camera model
        self.cam_model = PinholeCameraModel()

        # Variables to hold point cloud data
        self.point_cloud = None

        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_z = 0.0

        self.empty_patch = 0

        self.pos_x_lim = 0.3
        self.neg_x_lim = -0.3
        self.pos_y_lim = 2.0
        self.neg_y_lim = -2.0
        self.pos_z_lim = 0.0
        self.neg_z_lim = 6.0

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

        # Filter the point cloud to a region of interest
        filtered_points = np.array([[p[2], -p[0], p[1]] for p in downsampled_points if -0.3 <= p[1] <= 0.3 and -2 <= p[0] <= 2 and p[2] <= 6])
        clipped_points, self.empty_patch = self.clip_rectangle(filtered_points)

        if len(clipped_points) > 0 and self.empty_patch == 0:
            left_centers, right_centers = self.centroid(clipped_points)
            self.centroid_x, self.centroid_y, self.centroid_z = self.centre_of_line(left_centers, right_centers)
            self.publish_lines(left_centers, right_centers)
        else:
            self.centroid_x, self.centroid_y, self.centroid_z = 0.0, 0.0, 0.0

        self.empty_patch_pub.publish(self.empty_patch)
        self.publish_centroid_marker(self.centroid_x, self.centroid_y)
        self.publish_centroid(self.centroid_x, self.centroid_y, self.centroid_z)

    def centroid(self, points):
        # Separate points into left and right quadrants
        left_points = np.array([point for point in points if (point[1]) < 0])
        right_points = np.array([point for point in points if (point[1]) > 0])

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

    def clip_rectangle(self, points):
        # Clip points within a specified rectangular region and create markers
        clipped_points = []
        marker_array = MarkerArray()
        
        for idx, point in enumerate(points):
                clipped_points.append(point)
                # Create a cube marker for each clipped point
                marker = Marker()
                marker.header.frame_id = "camera_depth_frame"
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
        else:
            self.empty_patch = 0

        # Publish the array of markers
        self.clipped_marker_pub.publish(marker_array)
        return clipped_points, self.empty_patch

    def centre_of_line(self, left_centers, right_centers):
        if len(left_centers) > 0 and len(right_centers) > 0:
            center_left_quadrant = (left_centers[0] + left_centers[-1]) / 2
            center_right_quadrant = (right_centers[0] + right_centers[-1]) / 2
            waypoint = (center_left_quadrant + center_right_quadrant) / 2
            return waypoint[0], waypoint[1], waypoint[2]
        else:
            return 0.0, 0.0, 0.0

    def publish_centroid_marker(self, x, y):
        # Create and publish a marker for the centroid
        marker = Marker()
        marker.header.frame_id = "camera_depth_frame"
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

    def publish_centroid(self, x, y, z):
        # Publish the centroid position as a Pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        self.centroid_pub.publish(pose)

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
        marker.header.frame_id = "camera_depth_frame"
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

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create the node and run it
        node = LaneDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass