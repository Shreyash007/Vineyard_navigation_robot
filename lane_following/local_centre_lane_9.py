#!/usr/bin/env python3
from sklearn.cluster import DBSCAN
import rospy
import geometry_msgs
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
# from image_geometry import PinholeCameraModel # Not used in filtering/clustering logic
from sensor_msgs.msg import CameraInfo # Still needed for cam_model if used elsewhere

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, TransformStamped, PointStamped
from std_msgs.msg import Header
from std_msgs.msg import Int32
import tf2_ros
import tf2_geometry_msgs # Needed for transform methods

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=False)
        self.setup_parameters()

        # Subscribers
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

        # Publishers
        # self.centroid_marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=10) # Redundant with projected_waypoint_marker_pub
        self.centroid_pub = rospy.Publisher('/centroid', Pose, queue_size=10) # Waypoint Pose for controller
        self.empty_patch_pub = rospy.Publisher('/empty_patch', Int32, queue_size=10)
        self.lane_marker_array_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10) # Lane line visualization

        # Publisher for the projected point marker (waypoint)
        self.projected_waypoint_marker_pub = rospy.Publisher('/projected_waypoint_marker', Marker, queue_size=10)

        # Publisher for visualizing detected cluster centroids
        self.cluster_centroid_marker_pub = rospy.Publisher('/cluster_centroid_markers', MarkerArray, queue_size=10)

        # Publisher for visualizing the area of interest (filtering cuboid)
        self.filter_area_marker_pub = rospy.Publisher('/filter_area_marker', Marker, queue_size=10)


        # Set up tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) # Increased buffer size for robustness
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False # Flag to track if we have received odometry
        self.empty_patch = 0

        # Storing last valid lane centers (in odom frame) for persistence
        self.last_left_centroids_odom = []
        self.last_right_centroids_odom = []
        self.last_waypoint_pose = Pose() # Store last published waypoint pose (defaults to 0,0,0)


        # Load parameters
        self.load_dynamic_params(log_once=True)

        rospy.loginfo("Lane Detector node initialized.")


    def calculate_point_projection(self, x_0, y_0, x_1, y_1, x_c, y_c, d):
        """Calculate the projection of a point onto the line defined by (x0, y0) and (x1, y1)."""
        p1 = np.array([x_0, y_0])
        p2 = np.array([x_1, y_1])
        p_c = np.array([x_c, y_c]) # Current robot position

        line_vector = p2 - p1
        line_length_sq = np.dot(line_vector, line_vector)

        # Handle the case where the lane line is just a single point or very short
        if line_length_sq < 1e-9:
             rospy.logwarn_throttle(2, "Lane line segment is too short (start==end). Cannot project.")
             return x_c, y_c, x_c, y_c # Return current robot position as projected point


        # Vector from the start of the line to the current position
        w = p_c - p1

        # Project w onto v (the line vector)
        t = np.dot(w, line_vector) / line_length_sq # dot(v, v) is line_length_sq

        # The closest point on the infinite line defined by p1 and p2 to p_c is p1 + t * v
        closest_point_on_line = p1 + t * line_vector

        # Now, extend from the closest point along the line direction by distance d
        v_hat = line_vector / np.linalg.norm(line_vector) # Unit vector in the direction of the lane line
        projected_point = closest_point_on_line + d * v_hat

        return projected_point[0], projected_point[1], closest_point_on_line[0], closest_point_on_line[1]


    def publish_projected_waypoint_marker(self, x, y, timestamp):
        """Publish marker for the projected waypoint."""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = timestamp
        marker.ns = "projected_waypoint"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0 # Assume projected point is on the ground plane
        marker.scale.x = 0.4 # Make it clearly visible and distinct
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0 # Green waypoint
        marker.color.b = 0.0
        self.projected_waypoint_marker_pub.publish(marker)


    def odom_callback(self, msg):
        """Handles Odometry messages to get robot position."""
        # Extract the robot's position from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True # Set flag when first Odom message is received


    def setup_parameters(self):
            """
            Set up parameters on the parameter server with default values
            This allows changing parameters using rosparam set command
            """
            # Point cloud filtering parameters (in camera_depth_frame coordinates)
            # Camera frame: x=right, y=down, z=forward
            rospy.set_param('~filter_x_min', -0.8) # Corresponds to width filter (left)
            rospy.set_param('~filter_x_max', 0.8)  # Corresponds to width filter (right)
            rospy.set_param('~filter_y_min', -0.3) # Corresponds to height filter (down)
            rospy.set_param('~filter_y_max', 0.3)  # Corresponds to height filter (up)
            rospy.set_param('~filter_z_min', 0.7)  # Corresponds to distance filter (near)
            rospy.set_param('~filter_z_max', 4.0)  # Corresponds to distance filter (far)

            # Clustering parameters (applied to filtered points in camera frame)
            rospy.set_param('~dbscan_epsilon', 0.15)
            rospy.set_param('~dbscan_min_samples', 50)

            # Downsample factor for point cloud
            rospy.set_param('~downsample_factor', 10) # Process 1/10th of points

            # Projection distance for waypoint
            rospy.set_param('~projection_distance', 2.0) # Distance ahead of the robot to project

            # Persistence of old lane data when no lanes are detected
            rospy.set_param('~use_last_lane_if_empty', True)


    def load_dynamic_params(self, log_once=False):
        """
        Load parameters from rosparam with default values
        These can be dynamically changed using rosparam set/get commands
        """
        # Point cloud filtering parameters (in camera_depth_frame coordinates: x=right, y=down, z=forward)
        self.filter_x_min = rospy.get_param('~filter_x_min', -0.8)
        self.filter_x_max = rospy.get_param('~filter_x_max', 0.8)
        self.filter_y_min = rospy.get_param('~filter_y_min', -0.3)
        self.filter_y_max = rospy.get_param('~filter_y_max', 0.3)
        self.filter_z_min = rospy.get_param('~filter_z_min', 0.7)
        self.filter_z_max = rospy.get_param('~filter_z_max', 4.0)

        # Clustering parameters (applied to filtered points in camera frame)
        self.dbscan_eps = rospy.get_param('~dbscan_epsilon', 0.15)
        self.dbscan_min_samples = rospy.get_param('~dbscan_min_samples', 50)

        # Downsample factor
        self.downsample_factor = rospy.get_param('~downsample_factor', 10)

        # Projection distance
        self.projection_distance = rospy.get_param('~projection_distance', 2.0)

        # Persistence of old lane data
        self.use_last_lane_if_empty = rospy.get_param('~use_last_lane_if_empty', True)


        # Log the current parameters
        if log_once:
            rospy.loginfo(f"Current Lane Detection Parameters:")
            rospy.loginfo(f"Filter X (Width): [{self.filter_x_min}, {self.filter_x_max}]")
            rospy.loginfo(f"Filter Y (Height): [{self.filter_y_min}, {self.filter_y_max}]")
            rospy.loginfo(f"Filter Z (Distance): [{self.filter_z_min}, {self.filter_z_max}]")
            rospy.loginfo(f"DBSCAN Epsilon: {self.dbscan_eps}")
            rospy.loginfo(f"DBSCAN Min Samples: {self.dbscan_min_samples}")
            rospy.loginfo(f"Downsample Factor: {self.downsample_factor}")
            rospy.loginfo(f"Projection Distance: {self.projection_distance}")
            rospy.loginfo(f"Use last lane if empty: {self.use_last_lane_if_empty}")


    def camera_info_callback(self, cam_info):
        """Receives camera intrinsic parameters. Not directly used in this logic."""
        pass


    def transform_point_to_odom(self, point_in_camera_frame, timestamp):
        """Transform a point from camera_depth_frame to odom frame using point cloud timestamp."""
        p_stamped = PointStamped()
        p_stamped.header.frame_id = "camera_depth_frame"
        p_stamped.header.stamp = timestamp # Use the timestamp of the point cloud data
        p_stamped.point.x = point_in_camera_frame[0]
        p_stamped.point.y = point_in_camera_frame[1]
        p_stamped.point.z = point_in_camera_frame[2]

        try:
            # Transform the point to the odom frame
            # Increased timeout for robustness
            p_transformed = self.tf_buffer.transform(p_stamped, "odom", rospy.Duration(1.0))
            return np.array([p_transformed.point.x, p_transformed.point.y, p_transformed.point.z])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Log TF errors, especially extrapolation
            # rospy.logwarn_throttle(1.0, f"TF Transform failed for point at {timestamp.to_sec():.4f}: {e}")
            return None
        except Exception as e:
             rospy.logerr_throttle(1.0, f"An unexpected error during TF transform: {e}")
             return None


    def publish_filter_area_marker(self, timestamp):
        """Publishes a cuboid marker visualizing the point cloud filtering area in camera_depth_frame."""
        marker = Marker()
        marker.header.frame_id = "camera_depth_frame" # IMPORTANT: Publish in the camera frame
        marker.header.stamp = timestamp # Use the point cloud timestamp
        marker.ns = "filter_area"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Calculate center of the cuboid in camera_depth_frame
        center_x = (self.filter_x_min + self.filter_x_max) / 2.0
        center_y = (self.filter_y_min + self.filter_y_max) / 2.0
        center_z = (self.filter_z_min + self.filter_z_max) / 2.0

        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0 # No rotation

        # Calculate dimensions of the cuboid
        scale_x = abs(self.filter_x_max - self.filter_x_min)
        scale_y = abs(self.filter_y_max - self.filter_y_min)
        scale_z = abs(self.filter_z_max - self.filter_z_min)

        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z

        marker.color.a = 0.15 # Semi-transparent (adjust alpha as needed)
        marker.color.r = 0.0
        marker.color.g = 1.0 # Green color for the area
        marker.color.b = 0.0

        self.filter_area_marker_pub.publish(marker)


    def pointcloud_callback(self, data):
        """Processes point cloud data to detect and track lanes."""
        # Load dynamic parameters
        self.load_dynamic_params()

        # Publish the filtering area cuboid marker in the camera frame
        self.publish_filter_area_marker(data.header.stamp)

        # Read points from the point cloud message
        try:
            cloud_points_gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            cloud_points = list(cloud_points_gen) # Convert generator to list
        except Exception as e:
            rospy.logerr_throttle(1.0, f"Error reading point cloud: {e}")
            self.empty_patch = 1
            self.empty_patch_pub.publish(self.empty_patch)
            if self.use_last_lane_if_empty:
                 self.publish_last_known_lanes_and_waypoint(data.header.stamp)
            else:
                 self.publish_empty_visualizations(data.header.stamp)
            return

        # rospy.logdebug_throttle(1.0, f"Received PointCloud with {len(cloud_points)} points.")

        # Filter and downsample points in the camera_depth_frame
        filtered_points_camera_frame = []
        # Use a slice for downsampling
        for i in range(0, len(cloud_points), self.downsample_factor):
            p = cloud_points[i]
            # Filtering based on parameters
            if (self.filter_x_min <= p[0] <= self.filter_x_max and # Filter on camera X (right/left)
                self.filter_y_min <= p[1] <= self.filter_y_max and # Filter on camera Y (down/up)
                self.filter_z_min <= p[2] <= self.filter_z_max): # Filter on camera Z (distance forward)
                filtered_points_camera_frame.append([p[0], p[1], p[2]]) # Keep original camera frame coords

        filtered_points_camera_frame = np.array(filtered_points_camera_frame)
        # rospy.logdebug_throttle(1.0, f"Filtered down to {len(filtered_points_camera_frame)} points.")


        current_left_centroids_camera = []
        current_right_centroids_camera = []
        current_all_centroids_odom = [] # To store successfully transformed centroids for visualization

        if len(filtered_points_camera_frame) > 0:
             # Separate points into left and right based on camera X coordinate
            left_mask = filtered_points_camera_frame[:, 0] < 0 # Points to the left of camera center (negative X)
            right_mask = filtered_points_camera_frame[:, 0] > 0 # Points to the right of camera center (positive X)

            left_points_camera = filtered_points_camera_frame[left_mask]
            right_points_camera = filtered_points_camera_frame[right_mask]

            # rospy.logdebug_throttle(1.0, f"Left filtered points: {len(left_points_camera)}, Right filtered points: {len(right_points_camera)}")

            # Process left side
            if len(left_points_camera) > 0:
                left_clusters = self.cluster_points(left_points_camera, data.header.stamp, "left")
                # rospy.logdebug_throttle(1.0, f"Found {len(left_clusters)} left clusters.")
                for cluster in left_clusters:
                    cluster_centroid_camera = np.mean(cluster, axis=0)
                    current_left_centroids_camera.append(cluster_centroid_camera)
                    # Attempt to transform to odom frame
                    odom_point = self.transform_point_to_odom(cluster_centroid_camera, data.header.stamp)
                    if odom_point is not None:
                         current_all_centroids_odom.append((odom_point, 'left'))
                         # rospy.logdebug_throttle(1.0, f"Transformed left centroid to odom: {odom_point}")
                    # else:
                         # rospy.logdebug_throttle(1.0, "Failed to transform a left centroid.")


            # Process right side
            if len(right_points_camera) > 0:
                right_clusters = self.cluster_points(right_points_camera, data.header.stamp, "right")
                # rospy.logdebug_throttle(1.0, f"Found {len(right_clusters)} right clusters.")
                for cluster in right_clusters:
                    cluster_centroid_camera = np.mean(cluster, axis=0)
                    current_right_centroids_camera.append(cluster_centroid_camera)
                     # Attempt to transform to odom frame
                    odom_point = self.transform_point_to_odom(cluster_centroid_camera, data.header.stamp)
                    if odom_point is not None:
                        current_all_centroids_odom.append((odom_point, 'right'))
                        # rospy.logdebug_throttle(1.0, f"Transformed right centroid to odom: {odom_point}")
                    # else:
                        # rospy.logdebug_throttle(1.0, "Failed to transform a right centroid.")

            # Publish markers for all successfully transformed cluster centroids
            self.publish_cluster_centroid_markers(current_all_centroids_odom, data.header.stamp)
            # rospy.logdebug_throttle(1.0, f"Successfully transformed {len([p for p,s in current_all_centroids_odom if s == 'left'])} left and {len([p for p,s in current_all_centroids_odom if s == 'right'])} right centroids to odom.")


            # Now, process the centroids that were successfully transformed to the odom frame
            current_left_centroids_odom = [p for p, side in current_all_centroids_odom if side == 'left']
            current_right_centroids_odom = [p for p, side in current_all_centroids_odom if side == 'right']

            # rospy.logdebug_throttle(1.0, f"Odom Left centroids: {len(current_left_centroids_odom)}, Odom Right centroids: {len(current_right_centroids_odom)}")


            if len(current_left_centroids_odom) > 0 and len(current_right_centroids_odom) > 0 and self.odom_received:
                # We have successfully detected and transformed clusters on both sides, AND have robot's odom

                # Sort centroids by their X coordinate (forward in odom frame) to find start/end
                current_left_centroids_odom.sort(key=lambda p: p[0]) # Assuming odom X is forward
                current_right_centroids_odom.sort(key=lambda p: p[0]) # Assuming odom X is forward

                # Use the closest (smallest X) and farthest (largest X) transformed centroids
                # to define the lane line segment for projection.
                left_start_odom = current_left_centroids_odom[0]
                left_end_odom = current_left_centroids_odom[-1]
                right_start_odom = current_right_centroids_odom[0]
                right_end_odom = current_right_centroids_odom[-1]

                # Calculate the start and end points of the center line segment in odom frame
                self.lane_start = (left_start_odom + right_start_odom) / 2.0
                self.lane_end = (left_end_odom + right_end_odom) / 2.0
                # rospy.logdebug_throttle(1.0, f"Lane segment in odom: Start={self.lane_start[:2]}, End={self.lane_end[:2]}")


                # Store current successfully transformed centroids for persistence
                self.last_left_centroids_odom = current_left_centroids_odom
                self.last_right_centroids_odom = current_right_centroids_odom


                # Calculate and publish projected waypoint
                # Robot position is available because of the `self.odom_received` check above
                projected_x, projected_y, _, _ = self.calculate_point_projection(
                    self.lane_start[0], self.lane_start[1],
                    self.lane_end[0], self.lane_end[1],
                    self.robot_x, self.robot_y,
                    self.projection_distance  # Use the dynamic parameter
                 )
                 # Publish waypoint Pose
                waypoint_pose = Pose()
                waypoint_pose.position.x = projected_x
                waypoint_pose.position.y = projected_y
                waypoint_pose.position.z = 0.0 # Assume waypoint is on the ground
                self.centroid_pub.publish(waypoint_pose)
                self.last_waypoint_pose = waypoint_pose # Store last published waypoint
                rospy.logdebug_throttle(1.0, f"Published new waypoint: ({projected_x:.2f}, {projected_y:.2f})") # Keep waypoint log


                 # Publish projected waypoint marker
                self.publish_projected_waypoint_marker(projected_x, projected_y, data.header.stamp)


                # Publish the visualized lane lines connecting all detected centroids in odom frame
                self.publish_lanes(current_left_centroids_odom, current_right_centroids_odom, data.header.stamp)

                self.empty_patch = 0 # Lanes detected

            else:
                # Condition for valid waypoint calculation not met
                # rospy.logdebug_throttle(1.0, "Conditions for new waypoint calculation not met.")
                # if not self.odom_received:
                #      rospy.logdebug_throttle(1.0, " - Waiting for robot odom.")
                # if len(current_left_centroids_odom) == 0:
                #      rospy.logdebug_throttle(1.0, " - No successfully transformed left centroids.")
                # if len(current_right_centroids_odom) == 0:
                #      rospy.logdebug_throttle(1.0, " - No successfully transformed right centroids.")

                self.empty_patch = 1 # No valid lane pair detected in current frame
                # Use last valid lane data if configured
                if self.use_last_lane_if_empty:
                    self.publish_last_known_lanes_and_waypoint(data.header.stamp)
                else:
                    self.publish_empty_visualizations(data.header.stamp) # Clear markers if not using persistence


        else:
            # No filtered points detected at all
            # rospy.logdebug_throttle(1.0, "No points passed filters.")
            self.empty_patch = 1
            # Use last valid lane data if configured
            if self.use_last_lane_if_empty:
                 self.publish_last_known_lanes_and_waypoint(data.header.stamp)
            else:
                 self.publish_empty_visualizations(data.header.stamp) # Clear markers if not using persistence


        self.empty_patch_pub.publish(self.empty_patch)

    def publish_last_known_lanes_and_waypoint(self, timestamp):
        """Publishes the last known valid lane markers and waypoint if no new lanes were detected."""
        # Check if there was any last known data
        if len(self.last_left_centroids_odom) > 0 and len(self.last_right_centroids_odom) > 0:
            rospy.logdebug_throttle(1.0, "Using last known lane data.")
            # Publish the last known lane lines (connecting all centroids)
            self.publish_lanes(self.last_left_centroids_odom, self.last_right_centroids_odom, timestamp)

            # Publish the last known projected waypoint marker and Pose
            self.publish_projected_waypoint_marker(self.last_waypoint_pose.position.x, self.last_waypoint_pose.position.y, timestamp)
            self.centroid_pub.publish(self.last_waypoint_pose)

            # Publish cluster centroid markers for the last known clusters
            last_all_centroids_odom = [(p, 'left') for p in self.last_left_centroids_odom] + [(p, 'right') for p in self.last_right_centroids_odom]
            self.publish_cluster_centroid_markers(last_all_centroids_odom, timestamp)
        else:
             rospy.logdebug_throttle(1.0, "No last known lane data to publish.")
             # If no last known data exists, just clear everything
             self.publish_empty_visualizations(timestamp)


    def publish_empty_visualizations(self, timestamp):
        """Publishes DELETEALL markers to clear previous visualizations."""
        # rospy.logdebug_throttle(1.0, "No lanes detected, attempting to clear visualizations.")

        # Create a DELETEALL marker
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "odom" # Frame doesn't strictly matter for DELETEALL action
        delete_all_marker.header.stamp = timestamp
        delete_all_marker.action = Marker.DELETEALL

        # Create MarkerArray with DELETEALL
        delete_array = MarkerArray()
        delete_array.markers.append(delete_all_marker)

        # Publish DELETEALL to all relevant MarkerArray topics
        self.lane_marker_array_pub.publish(delete_array)
        self.cluster_centroid_marker_pub.publish(delete_array)
        self.filter_area_marker_pub.publish(delete_all_marker) # DELETEALL also works for single Marker publishers


        # For single Marker publishers like the projected waypoint, publish a DELETE marker
        # Or, rely on the next message on that topic (e.g., last_waypoint_pose or a new one)
        # Let's publish a DELETE marker for robustness.
        delete_waypoint_marker = Marker()
        delete_waypoint_marker.header.frame_id = "odom" # Frame needs to match the original marker
        delete_waypoint_marker.header.stamp = timestamp
        delete_waypoint_marker.ns = "projected_waypoint" # Namespace needs to match
        delete_waypoint_marker.id = 0 # ID needs to match
        delete_waypoint_marker.action = Marker.DELETE
        self.projected_waypoint_marker_pub.publish(delete_waypoint_marker)

        # Publish a zero Pose or the last known one when clearing visualizations
        # Publishing the last known one is generally safer for controllers.
        self.centroid_pub.publish(self.last_waypoint_pose)
        # rospy.logdebug_throttle(1.0, "Published last known waypoint pose or 0,0,0 after clearing visualizations.")


    def cluster_points(self, points, timestamp, side):
        """
        Performs DBSCAN clustering on the given points in camera_depth_frame.
        Returns a list of numpy arrays, where each array is a cluster.
        Includes logging about clustering results.
        """
        num_points = len(points)
        if num_points < self.dbscan_min_samples:
            # rospy.logdebug_throttle(1.0, f"Not enough {side} points ({num_points}) for clustering (min_samples={self.dbscan_min_samples})")
            return [] # Not enough points to form a cluster

        try:
            db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
            labels = db.labels_

            clusters = []
            unique_labels = set(labels)
            for label in unique_labels:
                if label != -1:  # Ignore noise points
                    cluster_points = points[labels == label]
                    if len(cluster_points) >= self.dbscan_min_samples: # Ensure valid cluster size
                         clusters.append(cluster_points)

            # rospy.logdebug_throttle(1.0, f"DBSCAN found {len(clusters)} valid {side} clusters from {num_points} points.")
            return clusters
        except Exception as e:
             rospy.logerr_throttle(1.0, f"DBSCAN clustering failed for {side} points: {e}")
             return []


    def publish_cluster_centroid_markers(self, centroids_with_side_odom, timestamp):
        """Publishes markers for all detected cluster centroids in the odom frame."""
        marker_array = MarkerArray()
        marker_id = 0

        # Publish DELETEALL first to clear previous centroids
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "odom"
        delete_all_marker.header.stamp = timestamp
        delete_all_marker.action = Marker.DELETEALL
        # Do not append DELETEALL here if your publisher handles it.
        # Appending DELETEALL to the same array as ADD markers can have inconsistent behavior
        # depending on Rviz version. Publishing DELETEALL separately before the new markers
        # is more reliable. Let's rely on MarkerArray replacing for now, but keep in mind.


        # Sort centroids by x (forward in odom) for consistent visualization IDs
        centroids_with_side_odom.sort(key=lambda item: item[0][0])

        for centroid_odom, side in centroids_with_side_odom:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = timestamp
            marker.ns = f"cluster_centroid_{side}" # Using side in namespace helps Rviz separate them
            marker.id = marker_id # Unique ID within the namespace
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = centroid_odom[0]
            marker.pose.position.y = centroid_odom[1]
            marker.pose.position.z = 0.0 # Assume on ground for visualization
            marker.scale.x = 0.15 # Slightly larger than before
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            if side == 'left':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 # Blue for left
            else: # right
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0 # Orange for right
            marker_array.markers.append(marker)
            marker_id += 1

        # Publish the array of new markers
        self.cluster_centroid_marker_pub.publish(marker_array)


    def publish_lanes(self, left_points_odom, right_points_odom, timestamp):
        """
        Publishes visualized lane lines and the center line connecting the detected centroids.
        Takes lists of centroid points in the odom frame for left and right lanes.
        """
        line_array = MarkerArray()
        marker_id = 0

        # Publish DELETEALL first to clear previous lines
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "odom"
        delete_all_marker.header.stamp = timestamp
        delete_all_marker.action = Marker.DELETEALL
        # Do not append DELETEALL here


        # Create line strip for left lane connecting all centroids
        if len(left_points_odom) > 0:
            left_marker = Marker()
            left_marker.header.frame_id = "odom"
            left_marker.header.stamp = timestamp
            left_marker.ns = "left_lane_strip"
            left_marker.id = marker_id
            left_marker.type = Marker.LINE_STRIP
            left_marker.action = Marker.ADD
            left_marker.scale.x = 0.1 # Thicker line
            left_marker.color.a = 1.0
            left_marker.color.r = 1.0 # Red for left lane
            left_marker.color.g = 0.0
            left_marker.color.b = 0.0
            # Ensure points have Z=0 for visualization on the ground plane
            left_marker.points = [Point(p[0], p[1], 0) for p in left_points_odom]
            line_array.markers.append(left_marker)
            marker_id += 1

        # Create line strip for right lane connecting all centroids
        if len(right_points_odom) > 0:
            right_marker = Marker()
            right_marker.header.frame_id = "odom"
            right_marker.header.stamp = timestamp
            right_marker.ns = "right_lane_strip"
            right_marker.id = marker_id
            right_marker.type = Marker.LINE_STRIP
            right_marker.action = Marker.ADD
            right_marker.scale.x = 0.1 # Thicker line
            right_marker.color.a = 1.0
            right_marker.color.r = 0.0 # Green for right lane
            right_marker.color.g = 1.0
            right_marker.color.b = 0.0
             # Ensure points have Z=0 for visualization on the ground plane
            right_marker.points = [Point(p[0], p[1], 0) for p in right_points_odom]
            line_array.markers.append(right_marker)
            marker_id += 1

        # Create center line connecting midpoints of corresponding centroids
        # Connect points based on index after sorting by depth (assuming left/right_points_odom are sorted by X (forward))
        min_len = min(len(left_points_odom), len(right_points_odom))
        if min_len > 0:
            center_marker = Marker()
            center_marker.header.frame_id = "odom"
            center_marker.header.stamp = timestamp
            center_marker.ns = "center_lane_strip"
            center_marker.id = marker_id
            center_marker.type = Marker.LINE_STRIP
            center_marker.action = Marker.ADD
            center_marker.scale.x = 0.1 # Thicker line
            center_marker.color.a = 1.0
            center_marker.color.r = 0.0 # Blue for center line
            center_marker.color.g = 0.0
            center_marker.color.b = 1.0
            center_marker.points = []
            for i in range(min_len):
                 mid_point = (left_points_odom[i] + right_points_odom[i]) / 2.0 # Use 2.0 for float division
                 center_marker.points.append(Point(mid_point[0], mid_point[1], 0)) # Use z=0
            line_array.markers.append(center_marker)
            marker_id += 1

        # Publish the array of lines
        self.lane_marker_array_pub.publish(line_array)


    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneDetector()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane Detector node interrupted.")