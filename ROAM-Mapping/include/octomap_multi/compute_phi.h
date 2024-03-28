#ifndef COMPUTE_PHI_H
#define COMPUTE_PHI_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <octomap/octomap.h>
#include <octomap_multi/Octomap_multi.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <boost/shared_ptr.hpp>
#include <unordered_set>
#include <string>
#include <vector>
#include <cmath>


typedef octomap::SemanticOcTree<octomap::SemanticsLogOdds> SemanticOctree;
typedef octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds> SemanticsOcTreeNode;


class PhiComputeNode{
public:
    PhiComputeNode(ros::NodeHandle& nh);
    
    ~PhiComputeNode();
    
    void octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg);

    static void expandNodeRecurse(SemanticOctree* inc_octomap_ptr, SemanticsOcTreeNode* node, unsigned int depth);

protected:
    ros::NodeHandle nh_;
    std::string octomap_topic_;
    ros::Publisher phi_pub_;
    ros::Subscriber octomap_sub_;
    octomap::KeySet key_set_;
    std_msgs::Float32 phi_msg_;
    std::vector<SemanticOctree*> inc_octomap_ptr_vec_;
    std::vector<bool> inc_octomap_val_vec_;
};

#endif //COMPUTE_PHI_H
