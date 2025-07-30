#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import math
import os
from threading import Lock

class TrajectoryLogger:
    def __init__(self):
        rospy.init_node('trajectory_logger', anonymous=True)

        rospy.loginfo("Trajectory Logger node started.")

        # --- Parameters ---
        default_path = os.path.join(os.path.expanduser('~'), 'trajectory_log.csv')
        self.output_file_path = rospy.get_param('~output_path', default_path)
        rospy.loginfo(f"Logging trajectory to: {self.output_file_path}")

        # --- State Variables ---
        self.latest_centroid = None
        self.data_lock = Lock() # To prevent race conditions between callbacks

        # --- File Handling ---
        try:
            self.csv_file = open(self.output_file_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp',
                'desired_x', 'desired_y',
                'actual_x', 'actual_y',
                'tracking_error'
            ])
        except IOError as e:
            rospy.logerr(f"Failed to open file {self.output_file_path}: {e}")
            rospy.signal_shutdown("File I/O error")
            return

        # --- Simple, Independent Subscribers ---
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/centroid', PoseStamped, self.centroid_callback)

        # --- Shutdown Hook ---
        rospy.on_shutdown(self.shutdown_hook)

    def centroid_callback(self, msg):
        """
        This callback simply stores the most recent desired waypoint.
        """
        with self.data_lock:
            self.latest_centroid = msg

    def odom_callback(self, odom_msg):
        """
        This callback is the main trigger for logging data.
        """
        # Do nothing if we haven't received a centroid waypoint yet.
        if self.latest_centroid is None:
            rospy.logwarn_throttle(5.0, "Waiting for first /centroid message...")
            return

        with self.data_lock:
            # Make a local copy to ensure data consistency during calculations
            current_centroid = self.latest_centroid

        # Get timestamp from the odometry message
        timestamp = odom_msg.header.stamp.to_sec()

        # Extract desired position from the stored /centroid message
        desired_x = current_centroid.pose.position.x
        desired_y = current_centroid.pose.position.y

        # Extract actual position from the incoming /odometry/filtered message
        actual_x = odom_msg.pose.pose.position.x
        actual_y = odom_msg.pose.pose.position.y

        # Calculate the tracking error
        tracking_error = math.sqrt((desired_x - actual_x)**2 + (desired_y - actual_y)**2)
        
        # Optional: Print to console to confirm it's working
        # print(f"Logging - Error: {tracking_error:.3f} m")

        # Write the synchronized data to the CSV file
        self.csv_writer.writerow([
            timestamp,
            desired_x, desired_y,
            actual_x, actual_y,
            tracking_error
        ])

    def shutdown_hook(self):
        """
        Cleanly close the CSV file on node shutdown.
        """
        rospy.loginfo("Shutting down Trajectory Logger node.")
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo("Log file closed successfully.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectoryLogger()
        node.run()
    except rospy.ROSInterruptException:
        pass