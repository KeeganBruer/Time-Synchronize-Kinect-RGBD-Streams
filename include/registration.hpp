#ifndef REGISTRATIONHPP
#define REGISTRATIONHPP

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <std_msgs/Float32MultiArray.h>
#include <chrono>
#include <thread>         // std::thread
#include <signal.h>
#include <string>

void getTransformationMatrix(int *state, std::vector<std::list<PointCloud::Ptr>> &list, std::vector<PointCloud::Ptr>& recent_cloud1, std::vector<PointCloud::Ptr>& recent_cloud2, int i);

#endif
