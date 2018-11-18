
#include <pcl_msgs/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

#include "euclidean_cluster/cluster.h"

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

		n.param<float>("leafsize", leafsize, 0.08f); // rosparam setのときは"0.07" (数値)
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

		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);

		size_t npoints = cloud_filtered->points.size();
		filtered_xyz->points.resize(npoints);

		// z to 0
		for(size_t i = 0; i < npoints; ++i){
			filtered_xyz->points[i].x = cloud_filtered->points[i].x;
			filtered_xyz->points[i].y = cloud_filtered->points[i].y;
			filtered_xyz->points[i].z = 0;
		}

		// creating the OctTree object
		pcl::gpu::Octree::PointCloud cloud_device;
		cloud_device.upload(filtered_xyz->points);

		pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
		octree_device->setCloud(cloud_device);
		octree_device->build();

		// extract clusters
		std::vector<pcl::PointIndices> cluster_indices_gpu;
		pcl::gpu::EuclideanClusterExtraction gec;
		gec.setClusterTolerance(tolerance);
		gec.setMinClusterSize(min_cluster_size);
		gec.setMaxClusterSize(max_cluster_size);
		gec.setSearchMethod(octree_device);
		gec.setHostCloud(filtered_xyz);
		gec.extract(cluster_indices_gpu);

		size_t nclusters = cluster_indices_gpu.size();

		indices_pub.clusters.resize(nclusters);
		for(size_t i = 0; i < nclusters; ++i){
			pcl_msgs::PointIndices indices;
			pcl_conversions::moveFromPCL(cluster_indices_gpu[i], indices);
			indices_pub.clusters[i] = indices;
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

