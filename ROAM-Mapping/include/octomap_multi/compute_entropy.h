#ifndef COMPUTE_ENTROPY
#define COMPUTE_ENTROPY

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


class EntropyComputeNode{
public:
    EntropyComputeNode(ros::NodeHandle& nh);
    
    ~EntropyComputeNode();
    
    void octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg);

protected:
    ros::NodeHandle nh_;
    std::string octomap_topic_;
    ros::Publisher ent_pub_;
    ros::Subscriber octomap_sub_;
    octomap_multi::Named_float ent_msg_;
};

#endif //COMPUTE_ENTROPY
