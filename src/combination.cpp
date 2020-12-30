#include "combination.hpp"
#include <signal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int end = 0;
void backup_signal_callback_handler(int signum) {
	end = 1;
	printf("stopping combination\n");
	fflush(stdout);
}
/*
 *	Start Kinect Combination Thread
 *	Arguments:
 *		- State, used to control thread from parent process.
 *		- 4f Matrix that holds the transformation of the secondary pointclouds
 *		- list of Pointclouds.
 *		- list of pointcloud's mutex
 *		- list of frame ids per topic.
 *		- ROS Publisher, used to publish the processed kinect stream.
 *	Thread that preforms the time registration and interpolation on recived pointclouds.
 *
 */
void start_kinect_combination(int *state, Eigen::Matrix4f &trans_matrix, std::vector<std::list<PointCloud::Ptr>> &list, pthread_mutex_t &mutex1, std::vector<std::string> &frame_ids, ros::Publisher* pub) {
        PointCloud k1_cloud_in; //k1 and k2 pointers - convert to std:vector for handeling 2+ 
        PointCloud k2_cloud_in;
        signal(SIGINT, backup_signal_callback_handler);
	
	pthread_mutex_lock(&mutex1);
        std::list<PointCloud::Ptr>::iterator it = list[0].begin();
        k1_cloud_in = **it;
       	it++;
        k2_cloud_in = **it;
        pthread_mutex_unlock(&mutex1);

	std::uint64_t current_stamp = k1_cloud_in.header.stamp;
        pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud;
	while (*state != -1 && end != 1) {
		if (list[0].empty()) {
			continue;
		}
                
                //std::cout << trans_matrix << std::endl;
		std::uint64_t stamp1 =k1_cloud_in.header.stamp;
		std::uint64_t stamp2 =k2_cloud_in.header.stamp;
		
		printf("frameID: %s\nstamp 1: %ld\n\nframeID: %s\nstamp 2: %ld\n", k1_cloud_in.header.frame_id.c_str(), stamp1, k2_cloud_in.header.frame_id.c_str(), stamp2); fflush(stdout);
		
		if (stamp2 < current_stamp) {
			k1_cloud_in = **it;
			it++;
			k2_cloud_in = **it;
			stamp1 =k1_cloud_in.header.stamp;
                	stamp2 =k2_cloud_in.header.stamp;
		}
		
		if (frame_ids.at(1).compare(k2_cloud_in.header.frame_id) != 0) {
                        pcl::transformPointCloud(k2_cloud_in, k2_cloud_in, trans_matrix);
			printf("cloud 2 transformed\n\n\n");
                }
		if (frame_ids.at(1).compare(k1_cloud_in.header.frame_id) != 0) {
                	pcl::transformPointCloud(k1_cloud_in, k1_cloud_in, trans_matrix);
			printf("cloud 1 transformed\n\n\n");
		}
		mPtrPointCloud = k1_cloud_in;
                mPtrPointCloud += k2_cloud_in;


                sensor_msgs::PointCloud2 object_msg;
                pcl::toROSMsg(mPtrPointCloud,object_msg );

                pub->publish(object_msg);
		current_stamp += 100;
        }
}



