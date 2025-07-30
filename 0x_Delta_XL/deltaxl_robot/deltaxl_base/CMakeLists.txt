cmake_minimum_required(VERSION 3.0.2)
project(deltaxl_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface deltaxl_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater hardware_interface deltaxl_msgs roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

set(ros0xrobotLIB ${PROJECT_SOURCE_DIR}/lib/lib0xRobotCPPLibrary.so)

add_executable(deltaxl_node
  src/deltaxl_base.cpp
  src/deltaxl_hardware.cpp
  src/deltaxl_diagnostics.cpp)

target_link_libraries(deltaxl_node ${ros0xrobotLIB} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp(
  src/deltaxl_base.cpp
  src/deltaxl_hardware.cpp
  src/deltaxl_diagnostics.cpp
  include/deltaxl_base/deltaxl_diagnostics.h
  include/deltaxl_base/deltaxl_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS deltaxl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
