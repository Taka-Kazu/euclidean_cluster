
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include "euclidean_cluster/cluster.h"

namespace euclidean_cluster
{
	template <typename PointT>
	Cluster<PointT>::Cluster()
	: n("~"), pc_sub(new PointCloud)
	{
		n.param<std::string>("topic_name_sub", topic_name_sub, "/velodyne_obstacles");
		n.param<std::string>("topic_name_pub", topic_name_pub, "/cluster/indices");

		obstacle_subscriber
			= n.subscribe<>(topic_name_sub, 1, &Cluster::callback, this);

		cluster_publisher
			= n.advertise<euclidean_cluster::IndicesClusters>(topic_name_pub, 1);

		n.param<double>("tolerance", tolerance, 0.15);
		n.param<int>("min_cluster_size", min_cluster_size, 20);
		n.param<int>("max_cluster_size", max_cluster_size, 1600);
	}

	template <typename PointT>
	Cluster<PointT>::~Cluster() {}

	//private
	template <typename PointT>
	void Cluster<PointT>::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		pcl::fromROSMsg(*msg, *pc_sub);
		extract();
		publish();
	}

	template <typename PointT>
	void Cluster<PointT>::extract()
	{
		typename
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		tree->setInputCloud (pc_sub);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointT> ec;
		ec.setClusterTolerance (tolerance);
		ec.setMinClusterSize (min_cluster_size);
		ec.setMaxClusterSize (max_cluster_size);
		ec.setSearchMethod (tree);
		ec.setInputCloud (pc_sub);
		ec.extract (cluster_indices);

		ic_pub.clusters.clear();
		// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
		for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl_msgs::PointIndices indices;
			pcl_conversions::moveFromPCL(*it, indices);
			ic_pub.clusters.push_back(indices);
			// pcl::PointIndices indicess;
			// pcl_conversions::moveToPCL(indices, indicess); // in subscriber
		}
	}

	template <typename PointT>
	void Cluster<PointT>::publish()
	{
		cluster_publisher.publish(ic_pub);
	}
} // namespace euclidean_cluster

