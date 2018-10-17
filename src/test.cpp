
#include <ros/ros.h>
#include "euclidean_cluster/cluster.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_euclidean_cluster");
	std::cout << "test euclidean cluster" << std::endl;

	euclidean_cluster::Cluster<pcl::PointNormal> clusters;

	ros::spin();

	return 0;
}

