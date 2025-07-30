#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import geometry_msgs
import math
import tf
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Int32


class LaserScanToGlobalPoints:
    def __init__(self):
        rospy.init_node('move_to_points', anonymous=True)
        #rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/centroid', PoseStamped, self.centroid_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
        self.dist_laser_x = transform.transform.translation.x
        self.dist_laser_y = transform.transform.translation.y
        #print(self.dist_laser_x, self.dist_laser_y)

        # Initialize the robot's pose in the /odom frame
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_z = 0.0

        self.centre_left_x = 0.0
        self.centre_left_y = 0.0
        self.centre_right_x = 0.0
        self.centre_right_y = 0.0

        self.delta_x = 0.0
        self.delta_y = 0.0


    def odom_callback(self, msg):
        # Extract the robot's position and orientation from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract orientation in quaternion and convert it to Euler angles (roll, pitch, yaw)
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # Calculate the distance to the waypoint
        distance = math.sqrt(self.delta_x**2 + self.delta_y**2)

        # Calculate the angle to the waypoint
        target_angle = math.atan2(self.delta_y, self.delta_x)

        # Calculate the difference between current orientation and target angle
        angle_diff = target_angle - self.robot_yaw

        # Normalize angle_diff to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        # Initialize Twist message
        cmd_vel = Twist()
        # If the distance is greater than the threshold, move towards the waypoint
        if distance > 0.01 :
            cmd_vel.linear.x = 0.15
            cmd_vel.angular.z = 2.0* angle_diff
            print('Going in a straight line')

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd_vel)
        

    def centroid_callback(self, msg):
        # Calculate the difference between current position and waypoint
        self.delta_x = msg.pose.position.x 
        self.delta_y = msg.pose.position.y

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
