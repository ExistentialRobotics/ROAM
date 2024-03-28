#include <octomap_multi/compute_coverage.h>


CoverageComputeNode::CoverageComputeNode(ros::NodeHandle& nh): nh_(nh)
{
    nh_.getParam("octomap_topic", octomap_topic_);
    
    cov_pub_ = nh_.advertise<octomap_multi::Named_float>("coverage", 1, true);
    
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &CoverageComputeNode::octomapCallback, this, ros::TransportHints().tcpNoDelay());
    
    key_set_.clear();
}

CoverageComputeNode::~CoverageComputeNode() {}

void CoverageComputeNode::octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg)
{
	ROS_INFO("Coverage node: A new map received!");
	
	SemanticOctree* inc_octree = new SemanticOctree(octomap_msg->octomap.resolution);
	if (inc_octree){
        std::stringstream datastream;
        if (octomap_msg->octomap.data.size() > 0){
            datastream.write((const char*) &(octomap_msg->octomap.data[0]), octomap_msg->octomap.data.size());
            inc_octree->readData(datastream);
      	}
    }
	
	for(typename SemanticOctree::leaf_iterator it = inc_octree->begin_leafs(),
      end=inc_octree->end_leafs(); it!= end; ++it)
  {
  	octomap::OcTreeKey node_key = it.getKey();
  	node_key[2] = 0;
  	key_set_.insert(node_key);
  }
  
  float res = (float) octomap_msg->octomap.resolution;

	cov_msg_.value.data = ((float) key_set_.size()) * res * res;
	cov_msg_.agent_name = octomap_msg->agent_name;

	cov_pub_.publish(cov_msg_);
	ROS_INFO("Coverage node: computation finished!");
	
	key_set_.clear();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_coverage");
    ros::NodeHandle nh("~");
    CoverageComputeNode coverageComputeNode(nh);
    ros::spin();
    
    return 0;
}
