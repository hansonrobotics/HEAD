cmake_minimum_required(VERSION 2.8.3)
project(robots_config)

find_package(catkin REQUIRED
#  rospy
#  std_msgs
#  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES whatever
#  CATKIN_DEPENDS tf2
#  DEPENDS system_lib
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
find_package(catkin COMPONENTS rostest)
add_rostest(test/test_robot.test)

