#include "combination.hpp"
#include <signal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int end = 0;
float FPS = 60;
std::uint64_t msec_inc = (((float)1)/FPS) * 1000;
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
void start_kinect_combination(int *state, std::vector<std::list<PointCloud::Ptr>> &list, pthread_mutex_t &mutex1, ros::Publisher* pub) {
        PointCloud k1_cloud_in; //k1 and k2 pointers - convert to std:vector for handeling 2+ 
        PointCloud k2_cloud_in;
        signal(SIGINT, backup_signal_callback_handler);
	
	pthread_mutex_lock(&mutex1);
        k1_cloud_in = *list[0].front();
       	list[0].pop_front();
        k2_cloud_in = *list[0].front();
        pthread_mutex_unlock(&mutex1);

	std::uint64_t current_stamp = k1_cloud_in.header.stamp;
        pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud;
	while (*state != -1 && end != 1) {
		if (list[0].size() < 2) {
			continue;
		}
                
                //std::cout << trans_matrix << std::endl;
		std::uint64_t stamp1 =k1_cloud_in.header.stamp;
		std::uint64_t stamp2 =k2_cloud_in.header.stamp;
		
		
		pthread_mutex_lock(&mutex1);
		for(;stamp2 < current_stamp;) {
			if (list[0].size() < 2) {
				break;
			}
			k1_cloud_in = *list[0].front();
			list[0].pop_front();
			k2_cloud_in = *list[0].front();
			stamp1 =k1_cloud_in.header.stamp;
                	stamp2 =k2_cloud_in.header.stamp;
		}
		pthread_mutex_unlock(&mutex1);
		
		if (list[0].size() < 2) {
                        continue;
                }

		printf("frameID: %s\nstamp 1: %ld\n", k1_cloud_in.header.frame_id.c_str(), stamp1);
                printf("current stamp: %ld\n", current_stamp);
                printf("frameID: %s\nstamp 2: %ld\n", k2_cloud_in.header.frame_id.c_str(), stamp2);
                fflush(stdout);
		bool isCorrectSelection = (stamp1 < current_stamp && stamp2 > current_stamp);
		printf("%ld < %ld < %ld is %s\n\n", stamp1, current_stamp, stamp2, isCorrectSelection ? "true" : "false");
		mPtrPointCloud = k1_cloud_in;
                mPtrPointCloud += k2_cloud_in;

                sensor_msgs::PointCloud2 object_msg;
                pcl::toROSMsg(mPtrPointCloud,object_msg );

                pub->publish(object_msg);
		current_stamp += msec_inc;
        }
	std::exit(0);
}



