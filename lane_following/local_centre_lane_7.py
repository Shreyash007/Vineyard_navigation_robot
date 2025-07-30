#!/usr/bin/env python3
from sklearn.cluster import DBSCAN
import rospy
import geometry_msgs
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Int32
import tf2_ros

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=False)
        self.setup_parameters()
        
        # Original subscribers
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # Original publishers
        self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)
        self.centroid_pub = rospy.Publisher('/centroid', Pose, queue_size=10)
        # self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        self.empty_patch_pub = rospy.Publisher('/empty_patch', Int32, queue_size=10)
        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)

        # self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        
        # New publisher for projected point
        self.projected_point_pub = rospy.Publisher('/projected_point', Marker, queue_size=10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.width = 640
        self.height = 480
        self.cam_model = PinholeCameraModel()
        
        # Initialize variables
        self.point_cloud = None
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_z = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.empty_patch = 0
        self.left_centers = []
        self.right_centers = []
        self.lane_start = None
        self.lane_end = None
        self.past_timesteps = 300
        
        # Load parameters
        self.load_dynamic_params(log_once=True)

    def calculate_point_projection(self, x_0, y_0, x_1, y_1, x_c, y_c, d):
        """Calculate the projection of a point onto the lane line."""
        current_position = np.array([x_c, y_c])
        v = np.array([x_1 - x_0, y_1 - y_0])
        v_hat = v / (np.linalg.norm(v) + 1e-10)  # Add small epsilon to avoid division by zero
        
        m = v[1]/(v[0] + 1e-10)
        
        if abs(m) > 100:  # Nearly vertical line
            x_i = x_0
            y_i = current_position[1]
        else:
            c = y_0 - m*x_0
            x_i = (m*current_position[1] - m*c + current_position[0])/(m**2 + 1)
            y_i = m*x_i + c
        
        closest_point = np.array([x_i, y_i])
        x_d, y_d = np.sum([closest_point, d*v_hat], axis=0)
        
        return x_d, y_d, x_i, y_i

    def publish_projected_point(self, x, y):
        """Publish marker for the projected point."""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "projected_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.projected_point_pub.publish(marker)

    def odom_callback(self, msg):
        # Extract the robot's position and orientation from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        transform: TransformStamped = self.tf_buffer.lookup_transform('odom', 'camera_depth_frame', rospy.Time(0), rospy.Duration(1.0))
        self.dist_x = transform.transform.translation.x
        self.dist_y = transform.transform.translation.y

    def setup_parameters(self):
            """
            Set up parameters on the parameter server with default values
            This allows changing parameters using rosparam set command
            """
            # Point cloud filtering parameters
            rospy.set_param('~height_filter_min', -0.8)
            rospy.set_param('~height_filter_max', -0.1)
            rospy.set_param('~width_filter_min', -2)
            rospy.set_param('~width_filter_max', 2)
            
            # Clustering parameters
            rospy.set_param('~dbscan_epsilon', 0.1)
            rospy.set_param('~dbscan_min_samples', 10)
            
            # Downsample factor
            rospy.set_param('~downsample_factor', 2)


    def load_dynamic_params(self, log_once=False):
        """
        Load parameters from rosparam with default values
        These can be dynamically changed using rosparam set/get commands
        """
        # Point cloud filtering parameters
        self.height_filter_min = rospy.get_param('~height_filter_min', -0.3)
        self.height_filter_max = rospy.get_param('~height_filter_max', 0.3)
        self.width_filter_min = rospy.get_param('~width_filter_min', -0.8)
        self.width_filter_max = rospy.get_param('~width_filter_max', 0.8)
        
        # Clustering parameters
        self.dbscan_eps = rospy.get_param('~dbscan_epsilon', 0.15)
        self.dbscan_min_samples = rospy.get_param('~dbscan_min_samples', 50)
        
        # Downsample factor
        self.downsample_factor = rospy.get_param('~downsample_factor', 10)

        # Log the current parameters
        if log_once:
            rospy.loginfo(f"Current Lane Detection Parameters:")
            rospy.loginfo(f"Height Filter: [{self.height_filter_min}, {self.height_filter_max}]")
            rospy.loginfo(f"Width Filter: [{self.width_filter_min}, {self.width_filter_max}]")
            rospy.loginfo(f"DBSCAN Epsilon: {self.dbscan_eps}")
            rospy.loginfo(f"DBSCAN Min Samples: {self.dbscan_min_samples}")
            rospy.loginfo(f"Downsample Factor: {self.downsample_factor}")

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
        """Optimized pointcloud callback with reduced calculations"""
        # Only reload parameters if needed (could be moved to a timer callback)
        self.load_dynamic_params()

        # Read points directly with required fields only
        self.point_cloud = point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)

        # Combine filtering and transformation in one step
        filtered_points = []
        for p in self.point_cloud[::self.downsample_factor]:  # Use slice for downsampling instead of random
            if (self.height_filter_min <= p[1] <= self.height_filter_max and 
                self.width_filter_min <= p[0] <= self.width_filter_max and 
                0.7 <= p[2] <= 4):
                filtered_points.append([p[2], -p[0], -p[1]])
        
        filtered_points = np.array(filtered_points)
        clipped_points, a = self.clip_rectangle(filtered_points)

        if len(filtered_points) > 0:
            left_centers, right_centers = self.centroid(filtered_points)
            if len(left_centers) > 0 and len(right_centers) > 0:
                self.centroid_x, self.centroid_y, self.centroid_z = self.centre_of_line(left_centers, right_centers)
                if len(left_centers) > self.past_timesteps and len(right_centers) > self.past_timesteps:
                    self.publish_lanes(left_centers[-self.past_timesteps], left_centers[-1], right_centers[-self.past_timesteps], right_centers[-1])
                    self.empty_patch = 0
                else:
                    self.publish_lanes(left_centers[0], left_centers[-1], right_centers[0], right_centers[-1])
                    self.empty_patch = 0
            else:
                #@TODO: make changes so that previous timestep is published instead of zero
                self.centroid_x, self.centroid_y, self.centroid_z = 0.0, 0.0, 0.0
                self.empty_patch = 1
        else:
            self.centroid_x, self.centroid_y, self.centroid_z = 0.0, 0.0, 0.0
            self.empty_patch = 1

        self.empty_patch_pub.publish(self.empty_patch)
        # self.publish_centroid_marker(self.centroid_x, self.centroid_y)
        #self.publish_centroid(self.centroid_x, self.centroid_y, self.centroid_z)
        

    def centroid(self, points):
        """Optimized centroid calculation"""
        # Use boolean indexing instead of list comprehension
        left_mask = points[:, 1] < 0
        right_mask = points[:, 1] > 0
        
        left_points = points[left_mask]
        right_points = points[right_mask]
        
        # self.left_centers = []
        # self.right_centers = []

        if len(left_points) > 0:
            left_clusters = self.cluster_points(left_points)
            for cluster in left_clusters:
                self.left_centers.append(np.mean(cluster + np.array([self.dist_x, self.dist_y, 0]), axis=0))

        if len(right_points) > 0:
            right_clusters = self.cluster_points(right_points)
            for cluster in right_clusters:
                self.right_centers.append(np.mean(cluster + np.array([self.dist_x, self.dist_y, 0]), axis=0))
                    
        return self.left_centers, self.right_centers

    def cluster_points(self, points):
        # Use DBSCAN clustering algorithm with dynamic parameters
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = db.labels_

        # Group points by cluster
        self.clusters = []
        for label in set(labels):
            if label != -1:  # Ignore noise points labeled as -1
                cluster = points[labels == label].tolist()
                self.clusters.append(cluster)
        #print(len(self.clusters))
        return self.clusters

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
       # self.clipped_marker_pub.publish(marker_array)
        return clipped_points, self.empty_patch

    def centre_of_line(self, left_centers, right_centers):
        
        if len(left_centers) > 0 and len(right_centers) > 0:
            # Calculate center points
            if len(left_centers) > self.past_timesteps and len(right_centers) > self.past_timesteps:
                center_left = (left_centers[-self.past_timesteps] + left_centers[-1]) / 2
                center_right = (right_centers[-self.past_timesteps] + right_centers[-1]) / 2
                # Store lane start and end points for projection
                self.lane_start = np.array([(left_centers[-self.past_timesteps][0]+right_centers[0][0])/2, (left_centers[-self.past_timesteps][1]+right_centers[0][1])/2])
                self.lane_end = np.array([(left_centers[-1][0]+right_centers[-1][0])/2, (left_centers[-1][1]+right_centers[-1][1])/2])
            else:
                center_left = (left_centers[0] + left_centers[-1]) / 2
                center_right = (right_centers[0] + right_centers[-1]) / 2
                # Store lane start and end points for projection
                self.lane_start = np.array([(left_centers[0][0]+right_centers[0][0])/2, (left_centers[0][1]+right_centers[0][1])/2])
                self.lane_end = np.array([(left_centers[-1][0]+right_centers[-1][0])/2, (left_centers[-1][1]+right_centers[-1][1])/2])
            
            waypoint = (center_left + center_right) / 2
            
            # Store lane start and end points for projection
            # self.lane_start = np.array([(left_centers[0][0]+right_centers[0][0])/2, (left_centers[0][1]+right_centers[0][1])/2])
            # self.lane_end = np.array([(left_centers[-1][0]+right_centers[-1][0])/2, (left_centers[-1][1]+right_centers[-1][1])/2])
            
            # Calculate and publish projected point
            if hasattr(self, 'robot_x') and hasattr(self, 'robot_y'):
                projected_x, projected_y, closest_x, closest_y = self.calculate_point_projection(
                    self.lane_start[0], self.lane_start[1],
                    self.lane_end[0], self.lane_end[1],
                    self.robot_x, self.robot_y,
                    2  # Projection distance
                )
                self.publish_projected_point(projected_x, projected_y)
                self.publish_centroid(projected_x, projected_y, self.centroid_z)
            
            return waypoint[0], waypoint[1], waypoint[2]
        return 0.0, 0.0, 0.0
    

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

    def publish_centroid(self, x, y, z):
        # Publish the centroid position as a Pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        self.centroid_pub.publish(pose)

    # def publish_lines(self, left_centers, right_centers):
    #     # Create a MarkerArray to publish the lines
    #     line_array = MarkerArray()

    #     # Create lines for the left quadrant
    #     for i, center in enumerate(left_centers[:-1]):
    #         next_center = left_centers[i + 1]
    #         line_array.markers.append(self.create_line_marker(center, next_center, i, "left_lines"))

    #     # Create lines for the right quadrant
    #     for i, center in enumerate(right_centers[:-1]):
    #         next_center = right_centers[i + 1]
    #         line_array.markers.append(self.create_line_marker(center, next_center, i + 100, "right_lines"))

    #     # Publish the array of lines
    #     self.marker_pub.publish(line_array)

    def publish_lanes(self, left_start, left_end, right_start, right_end):
        """Simplified lane publishing with only start and end points"""
        line_array = MarkerArray()
        
        # Create single line for left lane
        left_marker = self.create_line_marker(left_start, left_end, 0, "left_line")
        line_array.markers.append(left_marker)
        
        # Create single line for right lane
        right_marker = self.create_line_marker(right_start, right_end, 1, "right_line")
        line_array.markers.append(right_marker)
        
        self.marker_pub.publish(line_array)

    def create_line_marker(self, start, end, marker_id, ns):
        """Optimized line marker creation"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Only add start and end points
        start_point = Point(x=start[0], y=start[1], z=0)
        end_point = Point(x=end[0], y=end[1], z=0)
        marker.points = [start_point, end_point]

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
