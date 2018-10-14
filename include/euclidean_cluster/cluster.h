
#ifndef __EUCLIDEAN_CLUSTER_H
#define __EUCLIDEAN_CLUSTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <euclidean_cluster/IndicesClusters.h>
// #include <boost/thread.hpp>

namespace euclidean_cluster
{
	template <typename PointT>
	class Cluster
	{
		public:
		using PointCloud = pcl::PointCloud<PointT>;
		using PointCloudPtr = typename PointCloud::Ptr;
		// typedef typename PointCloud::ConstPtr PointCloudConstPtr;

		Cluster();
		~Cluster();

		private:
		void callback(const sensor_msgs::PointCloud2::ConstPtr&);
		void extract();
		void publish();

		ros::NodeHandle n; // private node handle ("~")
		ros::Subscriber obstacle_subscriber; // subscribed pointcloud
		ros::Publisher indices_publisher; // cluster indices
		ros::Publisher dspoints_publisher; // downsampled pointcloud

		std::string topic_name_sub; // topic name of pointcloud to subscribe
		std::string tname_indices; // topic name cluster indices
		std::string tname_dspoints; // topic name downsampled pointcloud

		// clustering params
		float leafsize;
		double tolerance;
		int min_cluster_size;
		int max_cluster_size;

		PointCloudPtr pc_sub; // pointcloud subscribed
		euclidean_cluster::IndicesClusters indices_pub; // cluster indices
		PointCloudPtr cloud_filtered; // downsampled pointcloud
		// boost::mutex pt_mutex;
	};

	template class Cluster<pcl::PointXYZ>;
	template class Cluster<pcl::PointXYZI>;
	// template class Cluster<pcl::PointXYZRGBA>;
	template class Cluster<pcl::PointXYZRGB>;
	// template class Cluster<pcl::PointXY>;
	// template class Cluster<pcl::Normal>;
	template class Cluster<pcl::PointNormal>;
	// template class Cluster<pcl::PointXYZRGBNormal>;
	template class Cluster<pcl::PointXYZINormal>;
}

#endif

