#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

class PointCloudConverter:
    def __init__(self):
        rospy.init_node('pcl_listener', anonymous=True)
        
        # Variables to hold camera model and PointCloud data
        self.cam_model = PinholeCameraModel()
        self.point_cloud = None
        
        # Set the width and height of the image (replace with actual camera resolution)
        self.width = 640  # Example camera width
        self.height = 480  # Example camera height
        
        # Subscriber to the camera info for camera model
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # Subscriber to the point cloud topic
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        
        # Publisher for filtered point cloud data
        self.pub = rospy.Publisher('/filtered/points', PointCloud2, queue_size=10)

    def camera_info_callback(self, cam_info):
        # Set the camera model once we get the camera info
        self.cam_model.fromCameraInfo(cam_info)

    def pointcloud_callback(self, data):
        # Store the point cloud data
        self.point_cloud = point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)

        # Convert point cloud data to array format for easy manipulation
        xyz_array = np.array([[p.x, p.y, p.z] for p in self.point_cloud])

        # Define the middle third band for u and v (both horizontal and vertical)
        u_min, u_max = int(self.width / 3), int(2 * self.width / 3)
        v_min, v_max = int(self.height / 3), int(2 * self.height / 3)

        filtered_points = []

        # Loop through all the points and check if they fall in the middle third
        for idx, p in enumerate(self.point_cloud):
            u, v = idx % self.width, idx // self.width  # Convert 1D index to 2D (u, v) coordinates

            # Check if point is in the middle third of the image 
            if -0.3 <= p.y <=0.3 and -2 <= p.x <= 1 :
                # Apply the Z-range filter and add point if it meets the condition
                    filtered_points.append([p.x, p.y, p.z])
                    print(f"Point at u={u}, v={v}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

        # Publish the filtered point cloud
        if filtered_points:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = data.header.frame_id
            filtered_pc = point_cloud2.create_cloud_xyz32(header, filtered_points)
            self.pub.publish(filtered_pc)

if __name__ == "__main__":
    converter = PointCloudConverter()
    rospy.spin()
