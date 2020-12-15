#include "combination.hpp"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
/*
 *	Start Kinect Combination Thread
 *	Arguments:
 *		- State, used to control thread from parent process.
 *		- ROS Publisher, used to publish the processed kinect stream.
 *	Thread that preforms the time registration and interpolation on recived pointclouds.
 *
 */
void start_kinect_combination(int *state, Eigen::Matrix4f &trans_matrix, std::vector<std::list<PointCloud::Ptr>> &list, pthread_mutex_t &mutex1, ros::Publisher* pub) {
        PointCloud k1_cloud_in; //k1 and k2 pointers - convert to std:vector for handeling 2+ 
        PointCloud k2_cloud_in;

        pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud;
	while (!list[1].empty() && !list[2].empty()) {
                pthread_mutex_lock(&mutex1);
                k1_cloud_in = *list[1].front();
                k2_cloud_in = *list[2].front();
                //k_list[0].pop_front();
                list[1].pop_front();
                list[2].pop_front();
                pthread_mutex_unlock(&mutex1);
                //std::cout << trans_matrix << std::endl;

                pcl::transformPointCloud(k2_cloud_in, k2_cloud_in, trans_matrix);
                
		mPtrPointCloud = k1_cloud_in;
                mPtrPointCloud += k2_cloud_in;
                printf("broadcasting New Pointcloud with w: %d h: %d\n", mPtrPointCloud.width, mPtrPointCloud.height);
                fflush(stdout);


                sensor_msgs::PointCloud2 object_msg;
                pcl::toROSMsg(mPtrPointCloud,object_msg );

                pub->publish(object_msg);
                if (*state == -1)
                        return;
        }

}



