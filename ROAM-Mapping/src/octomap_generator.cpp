#include <octomap_multi/octomap_generator.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator(): octomap_(0.05), raycast_range_(1.){}

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator(){}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    // Perform voxel filter
    float voxel_flt_size = octomap_.getResolution();
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxel_flt_size, voxel_flt_size, voxel_flt_size);
    sor.filter (*cloud_filtered);
    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    octomap::point3d origin(static_cast<float>(sensorToWorld(0,3)),static_cast<float>(sensorToWorld(1,3)),static_cast<float>(sensorToWorld(2,3)));
    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    for(typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
            raycast_cloud.push_back(it->x, it->y, it->z);
    }
    // Do ray casting for points in raycast_range_
    if(raycast_cloud.size() > 0)
        octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, false, true);
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename) const
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open()){
        std::cout << "Writing octomap to " << filename << std::endl;
        octomap_.write(outfile);
        outfile.close();
        std::cout << "Occupancy tree written " << filename << std::endl;
        return true;
    }
    else {
        std::cout << "Could not open " << filename  << " for writing" << std::endl;
        return false;
    }
}

//Explicit template instantiation
template class OctomapGenerator<PCLCloud, OccupancyOctree>;
