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
#include "combination.hpp"

//PointCloud Pointer containing the two most recent clouds.
std::vector<PointCloud::Ptr> recent_cloud1;
std::vector<PointCloud::Ptr> recent_cloud2;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

/*
 *	On Kinnect Recieved Function
 *	Arguments:
 *		- PointCloud message from any kinect stream
 *		- Kinect Number, 1 for the first topic, 2 for the second, etc.
 *
 *	Callback that recieves pointclouds from any of the node's subscribed topics.
 *	Used to load pointclouds into a Master LinkedList structure.
 */
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
	ros::init(argc, argv, "sub_pcl");
	signal(SIGINT, signal_callback_handler);
   	if (argc < 1) {
		printf("Insufficient Arguments.\n");
		printf("Please provide number of kinect topics followed by the topics seperated by a space.\n");
		fflush(stdout);
		std::exit(1);
   	}

	int kinect_num = atoi(argv[1]);
	if (argc < kinect_num+1) {
		printf("Insufficient Topics\n");
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
   	
	for (int i = 1; i < topic_count; i++) {
   		std::thread(
			getTransformationMatrix, 
			&state, 
			std::ref(k_list),
	       		std::ref(trans_matrix),	
			i+1
		).detach();
   	}
   
   	ros::NodeHandle nh;
   	
	ros::Subscriber subscribers[topic_count];
   	for (int i = 0; i < topic_count; i++) { 
       		subscribers[i] = nh.subscribe<PointCloud>(
			kinect_topics[i], //topic to subscribe to
			20, //queue size before new messages are thrown out. 
			boost::bind(on_k_recieved, _1, i+1) //callback takes 1 message and the id of 
		);
   	}
   	
	ros::Publisher cloud_pub = nh.advertise<PointCloud>("/sync/camera/points", 1000);

   	while(state != -1){ //-1 is the terminate state.
		if (state == 3) {
        		printf("starting Kinect Combination and Interpolation\n");
			fflush(stdout);
			std::thread(
				start_kinect_combination, //Thread Function Name
				std::ref(trans_matrix),
				std::ref(k_list), //Refs for the Master LinkedList
				std::ref(mutex1),
				&state, //Reusing state to control child process as well.
				&cloud_pub //ROS Publisher for completed pointcloud stream.
			).detach();
        		state = 4; 
		}
        	ros::spinOnce();
   	}
}





