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
	signal(SIGINT, backup_signal_callback_handler);

	printf("Number of topics %ld\n", list.size());

	PointCloud k1_cloud_in[list.size()]; //k1 and k2 pointers - convert to std:vector for handeling 2+
        PointCloud k2_cloud_in[list.size()];	
	
	pthread_mutex_lock(&mutex1);
	for (int i = 0; i < list.size(); i++) {
		k1_cloud_in[i] = *list[i].front();
       		list[i].pop_front();
        	k2_cloud_in[i] = *list[i].front();
	}
	pthread_mutex_unlock(&mutex1);

	std::uint64_t current_stamp = k1_cloud_in[0].header.stamp;
	
        std::uint64_t stamp1[list.size()];
        std::uint64_t stamp2[list.size()];

        pcl::PointCloud<pcl::PointXYZ>  interpolatedCloud[list.size()];
	pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud;
	int breakLoop = 0;
	while (*state != -1 && end != 1) {
		breakLoop = 0;
		for (int i = 0; i < list.size(); i++) {
			if (list[i].size() < 2) {
				breakLoop = 1;
				break;
			}
		}
		if (breakLoop == 1)
			continue;
                
		for (int i = 0; i < list.size(); i++) {
			stamp1[i] =k1_cloud_in[i].header.stamp;
			stamp2[i] =k2_cloud_in[i].header.stamp;
		}
		
		
		pthread_mutex_lock(&mutex1);
		for (int i = 0; i < list.size(); i++) {
			for(;stamp2[i] < current_stamp;) {
				if (list[i].size() < 2) {
					break;
				}
				k1_cloud_in[i] = *list[i].front();
                		list[i].pop_front();
                		k2_cloud_in[i] = *list[i].front();

				stamp1[i] =k1_cloud_in[i].header.stamp;
                		stamp2[i] =k2_cloud_in[i].header.stamp;
			}
		}
		pthread_mutex_unlock(&mutex1);
		
		breakLoop = 0;
		for (int i = 0; i < list.size(); i++) {
                        if (list[i].size() < 2) {
                                breakLoop = 1;
				break;
                        }
                }
		if (breakLoop == 1)
                        continue;

		for (int i = 1; i < list.size(); i++) {
			//printf("%d \t \tframeID: %s\nstamp 1: %ld\n",i, k1_cloud_in[i].header.frame_id.c_str(), stamp1[i]);
                	//printf("current stamp: %ld\n", current_stamp);
                	//printf("%d \t \tframeID: %s\nstamp 2: %ld\n",i, k2_cloud_in[i].header.frame_id.c_str(), stamp2[i]);
                	//fflush(stdout);
			bool isCorrectSelection = (stamp1[i] < current_stamp && stamp2[i] > current_stamp);
			printf("%ld < %ld < %ld is %s\n\n", stamp1[i], current_stamp, stamp2[i], isCorrectSelection ? "true" : "false");
			interpolatedCloud[i] =  k1_cloud_in[i];
                	interpolatedCloud[i] += k2_cloud_in[i];
		}
		
		mPtrPointCloud = interpolatedCloud[0];
		for (int i = 1; i < list.size(); i++) {
                	mPtrPointCloud += interpolatedCloud[i];
		}
		
                sensor_msgs::PointCloud2 object_msg;
                pcl::toROSMsg(mPtrPointCloud,object_msg );

                pub->publish(object_msg);
		current_stamp += msec_inc;
        }
	std::exit(0);
}



