#ifndef COMBINATIONHPP
#define COMBINATIONHPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <chrono>
#include <string>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void start_kinect_combination(int *state, std::vector<std::list<PointCloud::Ptr>> &list, pthread_mutex_t &mutex1, ros::Publisher* pub);

#endif
