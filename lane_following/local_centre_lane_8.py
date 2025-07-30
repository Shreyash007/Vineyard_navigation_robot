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
from geometry_msgs.msg import Pose, PoseStamped, Point, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Int32
import tf2_ros

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=False)
        self.setup_parameters()
        
        # --- Original Subscribers ---
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # --- Original Publishers ---
        self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10)
        self.centroid_pub = rospy.Publisher('/centroid', PoseStamped, queue_size=10)
        self.empty_patch_pub = rospy.Publisher('/empty_patch', Int32, queue_size=10)
        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)
        self.projected_point_pub = rospy.Publisher('/projected_point', Marker, queue_size=10)
        
        # --- NEW: Publishers for Visualization Enhancements ---
        self.filter_area_marker_pub = rospy.Publisher('/filter_area_marker', Marker, queue_size=10)
        self.cluster_centroid_marker_pub = rospy.Publisher('/cluster_centroid_markers', MarkerArray, queue_size=10)

        # --- TF and Initialization (Unchanged) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.width = 640
        self.height = 480
        self.cam_model = PinholeCameraModel()
        
        self.point_cloud = None
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_z = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.dist_x = 0.0 # Will be populated by TF lookup
        self.dist_y = 0.0 # Will be populated by TF lookup
        self.empty_patch = 0
        self.left_centers = []
        self.right_centers = []
        self.lane_start = None
        self.lane_end = None
        self.past_timesteps = 300
        
        self.load_dynamic_params(log_once=True)

    def pointcloud_callback(self, data):
        """Processes point cloud, computes waypoint, and publishes visualizations."""
        self.load_dynamic_params()
        
        # --- NEW: Publish the filtering area marker ---
        self.publish_filter_area_marker(data.header.stamp)

        self.point_cloud = point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)

        filtered_points = []
        for p in self.point_cloud[::self.downsample_factor]:
            if (self.height_filter_min <= p[1] <= self.height_filter_max and 
                self.width_filter_min <= p[0] <= self.width_filter_max and 
                0.7 <= p[2] <= 4):
                filtered_points.append([p[2], -p[0], -p[1]])
        
        filtered_points = np.array(filtered_points)

        if len(filtered_points) > 0:
            # The centroid method now returns the new centroids found in this frame
            new_left_centroids, new_right_centroids = self.centroid(filtered_points)
            
            # --- NEW: Publish the cluster centroid markers ---
            self.publish_cluster_centroid_markers(new_left_centroids, new_right_centroids, data.header.stamp)
            
            if len(self.left_centers) > 0 and len(self.right_centers) > 0:
                self.centroid_x, self.centroid_y, self.centroid_z = self.centre_of_line(self.left_centers, self.right_centers, data.header.stamp)
                if len(self.left_centers) > self.past_timesteps and len(self.right_centers) > self.past_timesteps:
                    self.publish_lanes(self.left_centers[-self.past_timesteps], self.left_centers[-1], self.right_centers[-self.past_timesteps], self.right_centers[-1])
                    self.empty_patch = 0
                else:
                    self.publish_lanes(self.left_centers[0], self.left_centers[-1], self.right_centers[0], self.right_centers[-1])
                    self.empty_patch = 0
            else:
                self.centroid_x, self.centroid_y, self.centroid_z = 0.0, 0.0, 0.0
                self.empty_patch = 1
        else:
            self.centroid_x, self.centroid_y, self.centroid_z = 0.0, 0.0, 0.0
            self.empty_patch = 1

        self.empty_patch_pub.publish(self.empty_patch)

    def centroid(self, points):
        """Calculates centroids and returns ONLY the new ones found in this frame."""
        left_mask = points[:, 1] < 0
        right_mask = points[:, 1] > 0
        
        left_points = points[left_mask]
        right_points = points[right_mask]
        
        new_left_centroids_odom = []
        new_right_centroids_odom = []

        if len(left_points) > 0:
            left_clusters = self.cluster_points(left_points)
            for cluster in left_clusters:
                # Core logic from original file preserved
                centroid_odom = np.mean(cluster + np.array([self.dist_x, self.dist_y, 0]), axis=0)
                self.left_centers.append(centroid_odom)
                new_left_centroids_odom.append(centroid_odom)

        if len(right_points) > 0:
            right_clusters = self.cluster_points(right_points)
            for cluster in right_clusters:
                # Core logic from original file preserved
                centroid_odom = np.mean(cluster + np.array([self.dist_x, self.dist_y, 0]), axis=0)
                self.right_centers.append(centroid_odom)
                new_right_centroids_odom.append(centroid_odom)
        
        # Return the centroids found in this specific frame for visualization
        return new_left_centroids_odom, new_right_centroids_odom

    def odom_callback(self, msg):
        # Unchanged from original
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('odom', 'camera_depth_frame', rospy.Time(0), rospy.Duration(1.0))
            self.dist_x = transform.transform.translation.x
            self.dist_y = transform.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"TF lookup failed from odom to camera_depth_frame: {e}")

    # --- NEW HELPER FUNCTIONS FOR VISUALIZATION ---

    def publish_filter_area_marker(self, timestamp):
        """Publishes a cuboid marker visualizing the point cloud filtering area."""
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame" # IMPORTANT: Published in camera frame
        marker.header.stamp = timestamp
        marker.ns = "filter_area"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Center of the cuboid based on filtering parameters
        center_x = (self.width_filter_min + self.width_filter_max) / 2.0
        center_y = (self.height_filter_min + self.height_filter_max) / 2.0
        center_z = (0.7 + 4.0) / 2.0 # Using the hardcoded Z filter from callback

        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z
        marker.pose.orientation.w = 1.0

        # Dimensions of the cuboid
        marker.scale.x = abs(self.width_filter_max - self.width_filter_min)
        marker.scale.y = abs(self.height_filter_max - self.height_filter_min)
        marker.scale.z = abs(4.0 - 0.7)

        marker.color.a = 0.35 # Semi-transparent
        marker.color.r = 0.0
        marker.color.g = 1.0 
        marker.color.b = 0.0

        self.filter_area_marker_pub.publish(marker)

    def publish_cluster_centroid_markers(self, left_centroids, right_centroids, timestamp):
        """Publishes SPHERE markers for all detected cluster centroids."""
        marker_array = MarkerArray()
        marker_id = 0

        for centroid in left_centroids:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = timestamp
            marker.ns = "cluster_centroids"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(centroid[0], centroid[1], centroid[2])
            marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0  # Blue for left centroids
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            marker_id += 1

        for centroid in right_centroids:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = timestamp
            marker.ns = "cluster_centroids"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(centroid[0], centroid[1], centroid[2])
            marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0  # Yellow for right centroids
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            marker_id += 1
            
        self.cluster_centroid_marker_pub.publish(marker_array)

    # --- ALL OTHER FUNCTIONS (UNCHANGED FROM local_centre_lane_7.py) ---
    
    def setup_parameters(self):
        rospy.set_param('~height_filter_min', -0.8)
        rospy.set_param('~height_filter_max', -0.1)
        rospy.set_param('~width_filter_min', -2)
        rospy.set_param('~width_filter_max', 2)
        rospy.set_param('~dbscan_epsilon', 0.1)
        rospy.set_param('~dbscan_min_samples', 10)
        rospy.set_param('~downsample_factor', 2)

    def load_dynamic_params(self, log_once=False):
        self.height_filter_min = rospy.get_param('~height_filter_min', -0.3)
        self.height_filter_max = rospy.get_param('~height_filter_max', 0.3)
        self.width_filter_min = rospy.get_param('~width_filter_min', -0.8)
        self.width_filter_max = rospy.get_param('~width_filter_max', 0.8)
        self.dbscan_eps = rospy.get_param('~dbscan_epsilon', 0.15)
        self.dbscan_min_samples = rospy.get_param('~dbscan_min_samples', 50)
        self.downsample_factor = rospy.get_param('~downsample_factor', 10)
        if log_once:
            rospy.loginfo(f"Height Filter: [{self.height_filter_min}, {self.height_filter_max}]")
            rospy.loginfo(f"Width Filter: [{self.width_filter_min}, {self.width_filter_max}]")
            rospy.loginfo(f"DBSCAN Epsilon: {self.dbscan_eps}, Min Samples: {self.dbscan_min_samples}")

    def camera_info_callback(self, cam_info):
        self.cam_model.fromCameraInfo(cam_info)

    def cluster_points(self, points):
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = db.labels_
        clusters = []
        for label in set(labels):
            if label != -1:
                cluster = points[labels == label].tolist()
                clusters.append(cluster)
        return clusters
    
    def calculate_point_projection(self, x_0, y_0, x_1, y_1, x_c, y_c, d):
        current_position = np.array([x_c, y_c])
        v = np.array([x_1 - x_0, y_1 - y_0])
        v_hat = v / (np.linalg.norm(v) + 1e-10)
        m = v[1]/(v[0] + 1e-10)
        if abs(m) > 100:
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
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "projected_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.2, 0.2
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0, 1.0
        self.projected_point_pub.publish(marker)

    def centre_of_line(self, left_centers, right_centers, timestamp):
        if len(left_centers) > 0 and len(right_centers) > 0:
            if len(left_centers) > self.past_timesteps and len(right_centers) > self.past_timesteps:
                self.lane_start = np.array([(left_centers[-self.past_timesteps][0]+right_centers[0][0])/2, (left_centers[-self.past_timesteps][1]+right_centers[0][1])/2])
                self.lane_end = np.array([(left_centers[-1][0]+right_centers[-1][0])/2, (left_centers[-1][1]+right_centers[-1][1])/2])
            else:
                self.lane_start = np.array([(left_centers[0][0]+right_centers[0][0])/2, (left_centers[0][1]+right_centers[0][1])/2])
                self.lane_end = np.array([(left_centers[-1][0]+right_centers[-1][0])/2, (left_centers[-1][1]+right_centers[-1][1])/2])
            
            if hasattr(self, 'robot_x') and hasattr(self, 'robot_y'):
                projected_x, projected_y, closest_x, closest_y = self.calculate_point_projection(
                    self.lane_start[0], self.lane_start[1], self.lane_end[0], self.lane_end[1],
                    self.robot_x, self.robot_y, 2
                )
                self.publish_projected_point(projected_x, projected_y)
                self.publish_centroid(projected_x, projected_y, 0.0, timestamp)
            
            # This part seems to calculate a different centroid for a marker, let's keep it
            center_left = (left_centers[0] + left_centers[-1]) / 2
            center_right = (right_centers[0] + right_centers[-1]) / 2
            waypoint = (center_left + center_right) / 2
            return waypoint[0], waypoint[1], waypoint[2]
        return 0.0, 0.0, 0.0

    def publish_centroid(self, x, y, z, timestamp): # <--- ADD timestamp argument
        # Publish the centroid position as a PoseStamped message
        pose_stamped_msg = PoseStamped()
        
        # Fill the header
        pose_stamped_msg.header.stamp = timestamp
        pose_stamped_msg.header.frame_id = "odom" # Assuming waypoint is in odom frame
        
        # Fill the pose
        pose_stamped_msg.pose.position.x = x
        pose_stamped_msg.pose.position.y = y
        pose_stamped_msg.pose.position.z = z
        # Orientation can be left as default (0,0,0,1) since controller doesn't use it
        pose_stamped_msg.pose.orientation.w = 1.0
        
        self.centroid_pub.publish(pose_stamped_msg)


    def publish_lanes(self, left_start, left_end, right_start, right_end):
        line_array = MarkerArray()
        left_marker = self.create_line_marker(left_start, left_end, 0, "left_line")
        right_marker = self.create_line_marker(right_start, right_end, 1, "right_line")
        line_array.markers.append(left_marker)
        line_array.markers.append(right_marker)
        self.marker_pub.publish(line_array)

    def create_line_marker(self, start, end, marker_id, ns):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.a, marker.color.r = 1.0, 1.0
        start_point = Point(x=start[0], y=start[1], z=0.2)
        end_point = Point(x=end[0], y=end[1], z=0.2)
        marker.points = [start_point, end_point]
        return marker

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass