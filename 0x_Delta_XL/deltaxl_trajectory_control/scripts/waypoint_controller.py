#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class WaypointController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('waypoint_controller')

        # Parameters for the waypoint
        self.waypoint = (5.809184898011631, 0.07325287202805325)  # Example waypoint (x, y)
        self.threshold = 0.1  # Threshold distance to consider waypoint reached
        self.linear_speed = 0.5  # Constant linear speed
        self.angular_speed = 1.0  # Constant angular speed

        # Subscriber to the odometry data (ground truth)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Store current position and orientation
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0

    def odometry_callback(self, msg):
        # Extract position from Odometry message
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        print("Current position:", self.current_position)
        # Extract orientation (yaw) from Odometry message (assuming quaternion is in z-axis rotation)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

        self.control_to_waypoint()

    def control_to_waypoint(self):
        # Calculate the difference between current position and waypoint
        delta_x = self.waypoint[0] - self.current_position[0]
        delta_y = self.waypoint[1] - self.current_position[1]

        # Calculate the distance to the waypoint
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Calculate the angle to the waypoint
        target_angle = math.atan2(delta_y, delta_x)

        # Calculate the difference between current orientation and target angle
        angle_diff = target_angle - self.current_orientation

        # Normalize angle_diff to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Initialize Twist message
        cmd_vel = Twist()

        # If the distance is greater than the threshold, move towards the waypoint
        if distance > self.threshold:
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = self.angular_speed * angle_diff
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
        controller = WaypointController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
