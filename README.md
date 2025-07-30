![ROS-DISTRO: Noetic](https://img.shields.io/badge/ROS--DISTRO-Noetic-blue?style=flat-square&logo=ros&logoColor=white)
![ROS-VERSION: 1.16.0](https://img.shields.io/badge/ROS--VERSION-1.16.0-blue?style=flat-square&logo=ros&logoColor=white)

## TIH launch files
For launching gazebo model
Install following pacakges
```
sudo apt-get install ros-noetic-robot_localization
```


# DeltaXL Gazebo
This package enables using DeltaXL robot in gazebo. Separate launch file is provided to integrate UR5e cobot on DeltaXL robot.
To launch only the robot in simulation

```
roslaunch deltaxl_gazebo deltaXL_world.launch
```

For launching Rviz visualization 
```
roslaunch deltaxl_viz view_robot.launch
```

Right now using turtlebot3 teleop_key for giving rotation commands
```
rosrun turtlebot3_teleop turtlebot3_teleop_key 
```

A lane centering algorithm
```
rosrun deltaxl_trajectory_control local_centre_lane.py
```
Tag detection and apriltag launch files
[Apriltag ros repository](https://github.com/AprilRobotics/apriltag_ros)
```
ROS_NAMESPACE=camera/color rosrun image_proc image_proc
roslaunch apriltag_ros continuous_detection.launch
```



For adding realsense in the scene, used this [repository](https://github.com/issaiass/realsense2_description).
And followed [this](https://github.com/pal-robotics/realsense_gazebo_plugin/issues/7) thread

Added the following in deltaXL.urdf.xacro file
```
  <xacro:include filename="$(find deltaxl_description)/urdf/_d435.urdf.xacro" />
  <xacro:sensor_d435 parent="base_link" >
    <origin xyz="0.2 0 0.65" rpy="0 0 0" />
  </xacro:sensor_d435>
```
Made some frequent changes into rosparam file
```
# Set new height filter range
rosparam set /lane_detector/height_filter_min -0.4
rosparam set /lane_detector/height_filter_max -0.1

# Set new DBSCAN parameters
rosparam set /lane_detector/dbscan_epsilon 0.1
rosparam set /lane_detector/dbscan_min_samples 20

# Set new downsample factor
rosparam set /lane_detector/downsample_factor 5
```

Adding GPS plugins from [hector gazebo plugin](http://wiki.ros.org/hector_gazebo_plugins) and [this](https://github.com/issaiass/jetbot_description) repository.

Or you can also install the GPS plugin using:
```
sudo apt-get install ros-noetic-hector-gazebo-plugins
```

Added MTK3339 GPS in simulation using the above repository and made some extensive changes in the URDF description files.

```
  <!-- GPS -->
  <xacro:include filename="$(find deltaxl_description)/urdf/mtk3339.urdf.xacro" />  
  <xacro:sensor_gps name="mtk3339" topic_ns="mtk_gps" parent="base_link" >
      <origin xyz="-0.2 0 0.45" rpy="0 0 0" />
  </xacro:sensor_gps>
  <!-- GPS -->
```

Added april tags from [this](https://github.com/koide3/gazebo_apriltag) repository.
