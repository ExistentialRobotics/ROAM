#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <octomap_multi/octomap_generator_base.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
typedef octomap::OcTree OccupancyOctree;

template<class CLOUD, class OCTREE>
class OctomapGenerator: public OctomapGeneratorBase<OCTREE>
{
public:
    /**
     * \brief Constructor
     */
    OctomapGenerator();

    virtual ~OctomapGenerator();
    
    virtual void setRayCastRange(float raycast_range){raycast_range_ = raycast_range;}

    virtual void setClampingThresMin(float clamping_thres_min)
    {
        octomap_.setClampingThresMin(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
        octomap_.setClampingThresMax(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
        octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
        octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
        octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
        octomap_.setProbMiss(prob_miss);
    }

    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld);

    virtual OCTREE* getOctree(){return &octomap_;}

    /**
     * \brief Save octomap to a file.
     * \param filename The output filename
     */
    virtual bool save(const char* filename) const;

protected:
    OCTREE octomap_; ///<Templated octree instance
    float raycast_range_; ///<Max range for points to perform raycasting
};

#endif //OCTOMAP_GENERATOR_H
