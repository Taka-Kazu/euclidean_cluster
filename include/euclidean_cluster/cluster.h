
#ifndef __EUCLIDEAN_CLUSTER_H
#define __EUCLIDEAN_CLUSTER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace euclidean_cluster{
	typedef pcl::PointXYZ PointXYZ;
	typedef pcl::PointCloud<PointXYZ> pcXYZ;
	typedef pcXYZ::Ptr pcXYZPtr;

	class Cluster{
		public:
		Cluster();
		~Cluster();

		void callback();
		void publish();

		private:
		void extract();

		ros::Subscriber obstacle_subscriber;
		ros::Publisher cluster_publisher;
	};
}

#endif

