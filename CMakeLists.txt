cmake_minimum_required(VERSION 3.0.2)
project(spPCL)

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
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

add_executable(spPCL src/spPCL.cpp src/registration.cpp)
target_link_libraries(spPCL ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(spPCL spPCL_generate_messages_cpp)

