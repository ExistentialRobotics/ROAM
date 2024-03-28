#ifndef COMPUTE_COVERAGE
#define COMPUTE_COVERAGE

#include <ros/ros.h>
#include <octomap_multi/Named_float.h>
#include <octomap/octomap.h>
#include <octomap_multi/Octomap_multi.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <string>
#include <cmath>


typedef octomap::SemanticOcTree<octomap::SemanticsLogOdds> SemanticOctree;
typedef octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds> SemanticsOcTreeNode;


class CoverageComputeNode{
public:
    CoverageComputeNode(ros::NodeHandle& nh);
    
    ~CoverageComputeNode();
    
    void octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg);

protected:
    ros::NodeHandle nh_;
    std::string octomap_topic_;
    ros::Publisher cov_pub_;
    ros::Subscriber octomap_sub_;
    octomap_multi::Named_float cov_msg_;
    octomap::KeySet key_set_;
};

#endif //COMPUTE_COVERAGE
