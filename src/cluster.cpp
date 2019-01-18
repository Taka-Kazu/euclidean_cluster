
#include <pcl_msgs/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include "euclidean_cluster/cluster.h"
#include <pcl/filters/passthrough.h>

namespace euclidean_cluster
{
	template <typename PointT>
	Cluster<PointT>::Cluster()
	: n("~"), pc_sub(new PointCloud), cloud_filtered(new PointCloud)
	{
		n.param<std::string>("topic_name_sub", topic_name_sub, "/velodyne_obstacles");
		n.param<std::string>("topic_name_indices", tname_indices, "/cluster/indices");
		n.param<std::string>("topic_name_dspoints", tname_dspoints, topic_name_sub+"/downsampled");

		obstacle_subscriber
			= n.subscribe<sensor_msgs::PointCloud2>(topic_name_sub, 1, &Cluster::callback, this);

		indices_publisher
			= n.advertise<euclidean_cluster::IndicesClusters>(tname_indices, 1);

		dspoints_publisher
			= n.advertise<sensor_msgs::PointCloud2>(tname_dspoints, 1);

		n.param<float>("leafsize", leafsize, 0.09f); // rosparam setのときは数値で (like "0.07")
		n.param<double>("tolerance", tolerance, 0.15); // 大きくすると重くなる
		n.param<int>("min_cluster_size", min_cluster_size, 20);
		n.param<int>("max_cluster_size", max_cluster_size, 900);
	}

	template <typename PointT>
	Cluster<PointT>::~Cluster() {}

	//private
	template <typename PointT>
	void Cluster<PointT>::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		// boost::mutex::scoped_lock(pt_mutex);
		pcl::fromROSMsg(*msg, *pc_sub);
    // min_max
    /*
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(pc_sub);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2, 1.8);
    pass.setNegative(true);
    pass.filter(*pc_sub);
    */
		extract();
		publish();
	}

	template <typename PointT>
	void Cluster<PointT>::extract()
	{
		// downsample the subscribed pointcloud using a leaf size of 5cm
		pcl::VoxelGrid<PointT> vg;
		vg.setInputCloud(pc_sub);
		vg.setLeafSize(leafsize, leafsize, leafsize);
		vg.filter(*cloud_filtered);
		// *cloud_filtered = *pc_sub;

		// z to 0
		size_t npoints = cloud_filtered->points.size();
		std::vector<double> org_z(npoints);
		for(size_t i = 0; i < npoints; ++i){
			org_z[i] = cloud_filtered->points[i].z;
			cloud_filtered->points[i].z = 0;
		}

		// creating the KdTree object
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		tree->setInputCloud(cloud_filtered);

		// extract clusters
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointT> ec;
		ec.setClusterTolerance(tolerance);
		ec.setMinClusterSize(min_cluster_size);
		ec.setMaxClusterSize(max_cluster_size);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_filtered);
		ec.extract(cluster_indices);

		// restore to original z
		for(size_t i = 0; i < npoints; ++i){
			cloud_filtered->points[i].z = org_z[i];
		}

		indices_pub.clusters.clear();
		// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
		for(auto it : cluster_indices){
			pcl_msgs::PointIndices indices;
			pcl_conversions::moveFromPCL(it, indices);
			indices_pub.clusters.push_back(indices);
		}
		pcl_conversions::fromPCL(pc_sub->header, indices_pub.header);
	}

	template <typename PointT>
	void Cluster<PointT>::publish()
	{
		// std::cout << "indices : " << indices_pub.clusters[0] << std::endl;
		// std::cout << "points : " << cloud_filtered->points[0].x << std::endl;
		indices_publisher.publish(indices_pub);

		sensor_msgs::PointCloud2 pc2;
		pcl::toROSMsg(*cloud_filtered, pc2);
		dspoints_publisher.publish(pc2);
	}

	// void toPCL(const IndicesClusters& ic, std::vector<pcl::PointIndices>& vec)
	// {
	// 	for(auto cluster : ic.clusters){
	// 		pcl::PointIndices indices;
	// 		pcl_conversions::moveToPCL(cluster, indices);
	// 		vec.push_back(indices);
	// 	}
	// }
    //
	// void fromPCL(const std::vector<pcl::PointIndices>& vec, IndicesClusters& ic)
	// {
	// 	for(auto indices : vec){
	// 		pcl_msgs::PointIndices cluster;
	// 		pcl_conversions::moveFromPCL(indices, cluster);
	// 		ic.clusters.push_back(cluster);
	// 	}
	// }
} // namespace euclidean_cluster

