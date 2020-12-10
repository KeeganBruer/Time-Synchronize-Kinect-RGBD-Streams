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
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// SIFT Keypoint parameters
const float min_scale = 0.01f; // the standard deviation of the smallest scale in the scale space
const int n_octaves = 3;  // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave = 4; // the number of scales to compute within each octave
const float min_contrast = 0.001f; // the minimum contrast required for detection

// Sample Consensus Initial Alignment parameters (explanation below)
const float min_sample_dist = 0.025f;
const float max_correspondence_dist = 0.01f;
const int nr_iters = 500;

// ICP parameters (explanation below)
const float max_correspondence_distance = 0.05f;
const float outlier_rejection_threshold = 0.05f;
const float transformation_epsilon = 0;
const int max_iterations = 100;


void getTransformationMatrix(int *state, std::vector<std::list<PointCloud::Ptr>> &list, Eigen::Matrix4f& trans_matrix, int i);


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;

Eigen::Matrix4f computeInitialAlignment (
	const PointCloudPtr & source_points, 
	const LocalDescriptorsPtr & source_descriptors,
        const PointCloudPtr & target_points, 
	const LocalDescriptorsPtr & target_descriptors,
        float min_sample_distance, 
	float max_correspondence_distance, 
	int nr_iterations
);

#endif
