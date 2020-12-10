#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <std_msgs/Float32MultiArray.h>
#include <chrono>
#include <thread>         // std::thread
#include <signal.h>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//transformation matrix to align k1 to k2.
Eigen::Matrix4f trans_matrix;

//List of pointers storing two kinect inputs streams as linked list.
//to support more kinects, needs to be array of linked lists.
std::vector<std::list<PointCloud::Ptr>> k_list;

#include "registration.hpp"

//PointCloud Pointer containing the two most recent clouds.
std::vector<PointCloud::Ptr> recent_cloud1;
std::vector<PointCloud::Ptr> recent_cloud2;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

void on_k_recieved(const PointCloud::ConstPtr& msg, int k_num)
{
  //printf ("Cloud recieved from k%d: width = %d, height = %d\n",k_num, msg->width, msg->height);
  PointCloud::Ptr p (new PointCloud);
  *p = *msg;
  pthread_mutex_lock(&mutex1);
  k_list[k_num].push_back(p);
  if (k_list[k_num].size() > 50) {
	k_list[k_num].pop_front();
  }
  pthread_mutex_unlock(&mutex1);
}

void start_kinect_combination(int *state, ros::Publisher* pub) {
	PointCloud k1_cloud_in;
	PointCloud k2_cloud_in;
	PointCloud::Ptr k2_cloud_trans(new PointCloud());
	pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud;
	while (!k_list[1].empty() && !k_list[2].empty()) {
		pthread_mutex_lock(&mutex1);
		k1_cloud_in = *k_list[1].front();
		k2_cloud_in = *k_list[2].front();
		//k_list[0].pop_front();
		k_list[1].pop_front();
		k_list[2].pop_front();
		pthread_mutex_unlock(&mutex1);
		//std::cout << trans_matrix << std::endl;
		pcl::transformPointCloud(k2_cloud_in, *k2_cloud_trans, trans_matrix);
		mPtrPointCloud = k1_cloud_in;
		mPtrPointCloud += *k2_cloud_trans;	
		printf("broadcasting New Pointcloud with w: %d h: %d\n", mPtrPointCloud.width, mPtrPointCloud.height);
		fflush(stdout);
		

		sensor_msgs::PointCloud2 object_msg;
		pcl::toROSMsg(mPtrPointCloud,object_msg );

		pub->publish(object_msg);
		if (*state == -1)
			return;
	}

}

int state = 0;

void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   state = -1;
   std::exit(signum);
}

int findTopicsFromArguments(int kinect_num, int argc, char **argv, char**kinect_topics) {
  int size = 0; 
  for (int search_for_topic = 0; search_for_topic < kinect_num; search_for_topic++) {
  	std::string arg(argv[search_for_topic+2]);
        kinect_topics[search_for_topic] = (char*)malloc((arg.length() +1) * sizeof(char));
        strcpy(kinect_topics[search_for_topic], arg.c_str());
	size++;
  }
  return size;
}


int main(int argc, char** argv)
{
   printf("%d arguments\n", argc);
   fflush(stdout);
   int kinect_num = 0;
   if (argc > 1) {
   	printf("first argument %s\n", argv[1]);
	fflush(stdout);
	kinect_num = atoi(argv[1]);
   } else {
	printf("Insufficient Arguments.\n");
	fflush(stdout);
	std::exit(1);
   }

   char **kinect_topics = (char**)malloc(kinect_num * sizeof(char*));
   int topic_count = findTopicsFromArguments(kinect_num, argc, argv, kinect_topics);
   for (int i = 0; i < topic_count; i++) {
	printf("%s\n", kinect_topics[i]);
   }
   for (int i = 0; i < topic_count+1; i++) {
       std::list<PointCloud::Ptr> list;
       k_list.push_back(list);
   }
   printf("g");fflush(stdout);
   for (int i = 1; i < topic_count; i++) {
   	std::thread(
		getTransformationMatrix, 
		&state, 
		std::ref(k_list),
	       	std::ref(trans_matrix),	
		i+1
	).detach();
   }
   
   ros::init(argc, argv, "sub_pcl");
   ros::NodeHandle nh;
   ros::Subscriber subscribers[topic_count];
   for (int i = 0; i < topic_count; i++) { 
       subscribers[i] = nh.subscribe<PointCloud>(kinect_topics[i], 1, boost::bind(on_k_recieved, _1, i+1));
   }
   ros::Publisher  cloud_pub = nh.advertise<PointCloud>("/sync/camera/points", 1000);
   
   signal(SIGINT, signal_callback_handler);

   while(state != -1){ //-1 is the terminate state.

	if (state == 3) {
        	printf("starting Kinect Combination and Interpolation\n");
		fflush(stdout);
		std::thread(start_kinect_combination, &state, &cloud_pub).detach();
                state = 4; 
	}
        ros::spinOnce();
   }

}





