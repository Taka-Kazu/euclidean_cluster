
#ifndef __EUCLIDEAN_CLUSTER_INDICES_CONVERSION_H
#define __EUCLIDEAN_CLUSTER_INDICES_CONVERSION_H

#include "euclidean_cluster/IndicesClusters.h"

namespace euclidean_cluster
{
	void toPCL(const IndicesClusters&, std::vector<pcl::PointIndices>&);
	void fromPCL(const std::vector<pcl::PointIndices>&, IndicesClusters&);

	void toPCL(const IndicesClusters& ic, std::vector<pcl::PointIndices>& vec)
	{
		vec.clear();
		for(auto cluster : ic.clusters){
			pcl::PointIndices indices;
			pcl_conversions::moveToPCL(cluster, indices);
			vec.push_back(indices);
		}
	}

	void fromPCL(const std::vector<pcl::PointIndices>& vec, IndicesClusters& ic)
	{
		ic.clusters.clear();
		if(vec.size()){
			pcl_conversions::fromPCL(vec[0].header, ic.header);
		}
		for(auto indices : vec){
			pcl_msgs::PointIndices cluster;
			pcl_conversions::moveFromPCL(indices, cluster);
			ic.clusters.push_back(cluster);
		}
	}
}

#endif

