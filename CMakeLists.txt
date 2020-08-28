cmake_minimum_required(VERSION 3.0.2)
project(ros_pygame_radar_2D)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


add_message_files(
   FILES
   PointPolar.msg
   RadarPointCloud.msg
)

catkin_python_setup() 


generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_pubsub_own_msg
   CATKIN_DEPENDS message_runtime rospy
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)