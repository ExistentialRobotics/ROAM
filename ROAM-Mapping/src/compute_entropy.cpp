#include <octomap_multi/compute_entropy.h>


EntropyComputeNode::EntropyComputeNode(ros::NodeHandle& nh): nh_(nh)
{
    nh_.getParam("octomap_topic", octomap_topic_);
    
    ent_pub_ = nh_.advertise<octomap_multi::Named_float>("entropy", 1, true);
    
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &EntropyComputeNode::octomapCallback, this, ros::TransportHints().tcpNoDelay());
}

EntropyComputeNode::~EntropyComputeNode() {}

void EntropyComputeNode::octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg)
{
	SemanticOctree* inc_octree = new SemanticOctree(octomap_msg->octomap.resolution);
	if (inc_octree){
        std::stringstream datastream;
        if (octomap_msg->octomap.data.size() > 0){
            datastream.write((const char*) &(octomap_msg->octomap.data[0]), octomap_msg->octomap.data.size());
            inc_octree->readData(datastream);
      	}
    }
    
    float entropy = 0;
    int num_leaf = 0;
    for(typename SemanticOctree::leaf_iterator it = inc_octree->begin_leafs(),
		        end=inc_octree->end_leafs(); it!= end; ++it)
	{
		float p = it->getOccupancy();
		entropy -= p * log(p) + (1 - p) * log(1 - p);
		num_leaf += 1;
	}
	
	ent_msg_.value.data = entropy / num_leaf;
	ent_msg_.agent_name = octomap_msg->agent_name;
	
	ent_pub_.publish(ent_msg_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_entropy");
    ros::NodeHandle nh("~");
    EntropyComputeNode entropyComputeNode(nh);
    ros::spin();
    
    return 0;
}
