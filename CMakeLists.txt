cmake_minimum_required(VERSION 3.0.2)
project(time_sync_kinects)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
)
find_package(Eigen3 REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES registration
   CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
   DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_LIBRARIES}
  ${Eigen3_INCLUDE_DIRS}
  src
  src/registration
  src/data_collection
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

add_executable(time_sync_kinects 
	src/registration/time_sync_kinects.cpp 
	src/registration/registration.cpp 
	src/registration/combination.cpp
	src/data_collection/tf_broadcaster.py
)
target_link_libraries(time_sync_kinects ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(time_sync_kinects time_sync_kinects_generate_messages_cpp)

