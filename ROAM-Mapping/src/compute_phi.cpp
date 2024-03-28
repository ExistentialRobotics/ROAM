#include <octomap_multi/compute_phi.h>


PhiComputeNode::PhiComputeNode(ros::NodeHandle& nh): nh_(nh)
{
    nh_.getParam("octomap_topic", octomap_topic_);
    
    int num_agents;
    nh_.getParam("num_agents", num_agents);
    
    phi_pub_ = nh_.advertise<std_msgs::Float32>("phi", 1, true);
    
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &PhiComputeNode::octomapCallback, this, ros::TransportHints().tcpNoDelay());

    key_set_.clear();
    inc_octomap_ptr_vec_ = std::vector<SemanticOctree*>(num_agents, nullptr);
    inc_octomap_val_vec_ = std::vector<bool>(num_agents, false);
}

PhiComputeNode::~PhiComputeNode() {}

void PhiComputeNode::octomapCallback(const octomap_multi::Octomap_multi::ConstPtr& octomap_msg)
{
    const char* agent_name = octomap_msg->agent_name.c_str();
    ROS_INFO("Compute phi node: A map received from %s!", agent_name);
    int robot_ind = int(agent_name[std::strlen(agent_name) - 1] - '0') - 1;
    inc_octomap_ptr_vec_[robot_ind] = new SemanticOctree(octomap_msg->octomap.resolution);
    inc_octomap_val_vec_[robot_ind] = true;
    if (inc_octomap_ptr_vec_[robot_ind]){
        std::stringstream datastream;
        if (octomap_msg->octomap.data.size() > 0){
            datastream.write((const char*) &(octomap_msg->octomap.data[0]), octomap_msg->octomap.data.size());
            inc_octomap_ptr_vec_[robot_ind]->readData(datastream);
      }
    }
    
    bool all_init = true;
    for (bool init : inc_octomap_val_vec_)
    {
    	all_init = all_init && init;
    	if (!all_init)
    		break;
    }
    
    if (all_init)
    {
    	ROS_INFO("Compute phi node: Octomap vetor full! Beginning computing phi...");
    	for (int i = 0; i < inc_octomap_ptr_vec_.size(); i++)
    	{
    		for(typename SemanticOctree::leaf_iterator it = inc_octomap_ptr_vec_[i]->begin_leafs(),
		        end=inc_octomap_ptr_vec_[i]->end_leafs(); it!= end; ++it)
		    {
		      unsigned int depth = it.getDepth();
		      SemanticsOcTreeNode& node = *it;
		      expandNodeRecurse(inc_octomap_ptr_vec_[i], &node, depth);
		    }
		    for(typename SemanticOctree::leaf_iterator it = inc_octomap_ptr_vec_[i]->begin_leafs(),
		        end=inc_octomap_ptr_vec_[i]->end_leafs(); it!= end; ++it)
		    {
		        key_set_.insert(it.getKey());
		    }
    	}
    	
    	float phi = 0;
    	for (auto& key : key_set_)
    		for (int i = 0; i < inc_octomap_ptr_vec_.size(); i++)
    			for (int j = i + 1; j < inc_octomap_ptr_vec_.size(); j++)
    			{
    				SemanticsOcTreeNode* node_i = inc_octomap_ptr_vec_[i]->search(key, 16);
		            SemanticsOcTreeNode* node_j = inc_octomap_ptr_vec_[j]->search(key, 16);
		            
		            float log_odds_i = 0, log_odds_j = 0;
		            
		            if (node_i)
		                log_odds_i = node_i->getValue();
		                
		            if (node_j)
		                log_odds_j = node_j->getValue();
		            
		            phi += pow(log_odds_i - log_odds_j, 2);
    			}
    	
    	phi_msg_.data = phi;
    	phi_pub_.publish(phi_msg_);
    	std::fill(inc_octomap_val_vec_.begin(), inc_octomap_val_vec_.end(), false);
    	ROS_INFO("Compute phi node: Computing phi completed!");
    }
    
}

void PhiComputeNode::expandNodeRecurse(SemanticOctree* inc_octomap_ptr, SemanticsOcTreeNode* node, unsigned int depth)
{
  if (depth != 16)
  {
    inc_octomap_ptr->expandNode(node);
    if (depth < 15)
    {
      for (unsigned int i=0; i<8; i++) {
        SemanticsOcTreeNode* child = inc_octomap_ptr->getNodeChild(node, i);
        expandNodeRecurse(inc_octomap_ptr, child, depth + 1);
      }
    }
  }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_phi");
    ros::NodeHandle nh("~");
    PhiComputeNode phiComputeNode(nh);
    ros::spin();
    
    return 0;
}
