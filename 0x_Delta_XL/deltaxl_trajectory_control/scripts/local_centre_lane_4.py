#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
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
        rospy.Subscriber('/centroid', Pose, self.centroid_callback)
        rospy.Subscriber('/empty_patch', Int32, self.patch)
        self.marker_pub = rospy.Publisher('/lane_markers', MarkerArray, queue_size=10)
        # Publisher for the centroid marker
        #self.clipped_marker_pub = rospy.Publisher('/clipped_markers', MarkerArray, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'camera_depth_frame', rospy.Time(0), rospy.Duration(1.0))
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

        self.count = 0

        self.empty_patch_call = [0]
        self.waypoint = [[19.8,0.2],[19.8,-1.8]]

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
        if len(self.empty_patch_call)>4:
            for i in range(4):
                if self.empty_patch_call[-1-i]==1 and self.empty_patch_call[-1]==1:
                    self.count+=1
                    print('inside loop')
        print('count:', self.count)
        print('empty_patch:', self.empty_patch_call)
        if self.count>3 and self.empty_patch_call[-1]==1:
            # cmd_vel.linear.x = 0.05
            # cmd_vel.angular.z = -0.03333
            # print("Turning")
            # self.count=0
            self.follow_arc()

        # If the distance is greater than the threshold, move towards the waypoint
        if distance > 0.01 and self.empty_patch_call[-1]==0 and self.empty_patch_call[-2]==0:
            cmd_vel.linear.x = 0.15
            cmd_vel.angular.z = 0.3 * angle_diff
            print('Going in a straight line')

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def patch(self, msg):
        print(msg.data)
        self.empty_patch_call.append(msg.data)
        

    def centroid_callback(self, msg):
        # Calculate the difference between current position and waypoint
        self.delta_x = msg.position.x - self.robot_x
        self.delta_y = msg.position.y - self.robot_y


    def follow_arc(self):
        self.start_point = self.waypoint[0]
        self.end_point = self.waypoint[1]    
        self.radius = 1.6            
        self.angle_step = 0.05
        start_angle = math.atan2(self.end_point[1] - self.start_point[1], self.end_point[0] - self.start_point[0])
        arc_points = []
        
        # Generate arc points
        for angle in self.generate_arc(start_angle):
            x = self.start_point[0] + self.radius * math.cos(angle)
            y = self.start_point[1] + self.radius * math.sin(angle)
            arc_points.append((x, y))
        
        # Move the robot along the arc points
        for point in arc_points:
            self.move_to_point(point)

    def generate_arc(self, start_angle):
        angles = []
        end_angle = start_angle + math.pi / 2  # Adjust as necessary for desired arc direction
        angle = start_angle
        
        while angle <= end_angle:
            angles.append(angle)
            angle += self.angle_step
        
        return angles

    def move_to_point(self, point):
        # Create a Twist message
        cmd = Twist()
        target_x, target_y = point
        rospy.loginfo(f'Moving to point: {point}')
        
        # Example control logic: Adjust speed based on distance to the target
        distance = math.sqrt((target_x - self.start_point[0])**2 + (target_y - self.start_point[1])**2)
        if distance > 0.06:
            cmd.linear.x = 0.05 
            cmd.angular.z = 0.4 * (math.atan2(target_y - self.start_point[1], target_x - self.start_point[0]) - self.robot_yaw)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd)



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
