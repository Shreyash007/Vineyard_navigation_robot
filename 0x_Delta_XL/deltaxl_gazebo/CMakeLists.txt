cmake_minimum_required(VERSION 2.8.3)
project(deltaxl_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY launch Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
