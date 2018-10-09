
#ifndef __EUCLIDEAN_CLUSTER_H
#define __EUCLIDEAN_CLUSTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_msgs/PointIndices.h>
#include <euclidean_cluster/IndicesClusters.h>

namespace euclidean_cluster
{
	template <typename PointT>
	class Cluster
	{
		public:
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;

		Cluster();
		~Cluster();

		private:
		void callback(const sensor_msgs::PointCloud2::ConstPtr&);
		void extract();
		void publish();

		ros::NodeHandle n;
		ros::Subscriber obstacle_subscriber;
		ros::Publisher cluster_publisher;

		std::string topic_name_sub;
		std::string topic_name_pub;

		// clustering params
		double tolerance;
		int min_cluster_size;
		int max_cluster_size;

		PointCloudPtr pc_sub;
		euclidean_cluster::IndicesClusters ic_pub;
	};

	template class Cluster<pcl::PointXYZ>;
	template class Cluster<pcl::PointNormal>;
	template class Cluster<pcl::PointXYZINormal>;
}

#endif

