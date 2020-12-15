#include "registration.hpp"
#include "combination.hpp"

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void getTransformationMatrix(int *state, std::vector<std::list<PointCloud::Ptr>> &list, Eigen::Matrix4f& trans_matrix, int i) {
  printf("Starting Thread %d to Find Transformation Matrix\n", i);
  fflush(stdout);
  pthread_mutex_lock(&mutex);
  int size1 = list[1].size();
  int size2 = list[i].size();
  pthread_mutex_unlock(&mutex);

  while (size1 == 0 || size2 == 0){
        printf("size1 %d size2 %d\n", size1, size2);
        fflush(stdout);
        if (*state == -1)
                std::exit(2);
        pthread_mutex_lock(&mutex);
        size1 = list[1].size();
        size2 = list[i].size();
        pthread_mutex_unlock(&mutex);
        if (size1 == 0 || size2 == 0)
                sleep(1);
  }
  PointCloud::Ptr recent_cloud1 = list[1].front();
  PointCloud::Ptr recent_cloud2 = list[i].front();
  list[1].pop_front();
  list[i].pop_front();

  printf("Kinect Cloud In Thread\n\n");
  fflush(stdout);
  PointCloud::Ptr filtered1 (new PointCloud);
  PointCloud::Ptr filtered2 (new PointCloud);

  pthread_mutex_lock(&mutex);
  //Filter out NaN and Inf from the most recent cloud.
  boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*recent_cloud1, *indices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(recent_cloud1);
  extract.setIndices(indices);
  extract.setNegative(false);

  extract.filter(*filtered1);

  //Filter out NaN and Inf from the second most recent cloud.
  boost::shared_ptr<std::vector<int>> indices2(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*recent_cloud2, *indices2);
  pcl::ExtractIndices<pcl::PointXYZ> extract2;
  extract2.setInputCloud(recent_cloud2);
  extract2.setIndices(indices2);
  extract2.setNegative(false);
  extract2.filter(*filtered2);

  printf("Attemping To Register\n");
  fflush(stdout);
  pthread_mutex_unlock(&mutex);


  if (*state == -1)
          std::exit(0);
  //Start registration

  /*
  //preform Pointcloud Registration using iteritive method.
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(filtered1);
  icp.setInputTarget(filtered2);
  
  //Call iterative algorithm to produce registered pointcloud.
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  //print out registration statistics.
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  std::cout << transformation << std::endl;
  */

  cout << "Computing source cloud normals\n";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(filtered2);
  ne.setSearchMethod(tree_xyz);
  ne.setRadiusSearch(0.05);
  ne.compute(*src_normals_ptr);
  for(size_t i = 0;  i < src_normals.points.size(); ++i) {
      src_normals.points[i].x = filtered2->points[i].x;
      src_normals.points[i].y = filtered2->points[i].y;
      src_normals.points[i].z = filtered2->points[i].z;
  }

  cout << "Computing target cloud normals\n";
  pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
  ne.setInputCloud(filtered1);
  ne.compute(*tar_normals_ptr);
  for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
      tar_normals.points[i].x = filtered1->points[i].x;
      tar_normals.points[i].y = filtered1->points[i].y;
      tar_normals.points[i].z = filtered1->points[i].z;
  }

  // Estimate the SIFT keypoints
  pcl::SIFTKeypoint<pcl::PointNormal, PointT> sift;
  pcl::PointCloud<PointT>::Ptr src_keypoints_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>& src_keypoints = *src_keypoints_ptr;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree_normal);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(src_normals_ptr);
  sift.compute(src_keypoints);

  cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";

  pcl::PointCloud<PointT>::Ptr tar_keypoints_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>& tar_keypoints = *tar_keypoints_ptr;
  sift.setInputCloud(tar_normals_ptr);
  sift.compute(tar_keypoints);

  cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";

  // Extract FPFH features from SIFT keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setSearchSurface (filtered2);
  fpfh.setInputCloud (src_keypoints_xyz);
  fpfh.setInputNormals (src_normals_ptr);
  fpfh.setSearchMethod (tree_xyz);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(src_features);
  cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
  fpfh.setSearchSurface (filtered1);
  fpfh.setInputCloud (tar_keypoints_xyz);
  fpfh.setInputNormals (tar_normals_ptr);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  fpfh.compute(tar_features);
  cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";

  // Compute the transformation matrix for alignment
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);

  //print out registration statistics.
  std::cout << transformation << std::endl;

  //end registration

  //Set global tranformation.
  trans_matrix = transformation;
  //move state to join thread.
  *state = 3;
  printf("Finished Finding Transformation Matrix\n");
  fflush(stdout);
  //std::exit(0);
}

Eigen::Matrix4f computeInitialAlignment (
	const PointCloudPtr & source_points, 
	const LocalDescriptorsPtr & source_descriptors,
	const PointCloudPtr & target_points, 
	const LocalDescriptorsPtr & target_descriptors,
        float min_sample_distance, 
	float max_correspondence_distance, 
	int nr_iterations
) {
  pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);

  sac_ia.setInputCloud (source_points);
  sac_ia.setSourceFeatures (source_descriptors);

  sac_ia.setInputTarget (target_points);
  sac_ia.setTargetFeatures (target_descriptors);

  PointCloud registration_output;
  sac_ia.align (registration_output);

  return (sac_ia.getFinalTransformation ());
}


std_msgs::Float32MultiArray matrix4f_to_float32MultiArray(Eigen::Matrix4f matrix) {
  std_msgs::Float32MultiArray msg;
  if (msg.layout.dim.size() != 2)
    msg.layout.dim.resize(2);
  msg.layout.dim[0].stride = matrix.rows() * matrix.cols();
  msg.layout.dim[0].size = matrix.rows();
  msg.layout.dim[1].stride = matrix.cols();
  msg.layout.dim[1].size = matrix.cols();
  if ((int)msg.data.size() != matrix.size())
    msg.data.resize(matrix.size());
  int ii = 0;
  for (int i = 0; i < matrix.rows(); ++i)
    for (int j = 0; j < matrix.cols(); ++j)
      msg.data[ii++] = matrix.coeff(i, j);

  return msg;
}



