#ifndef SEMANTIC_MULTI_OCTOMAP_GENERATOR_ROS_H
#define SEMANTIC_MULTI_OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <semantic_octomap_node/octomap_generator.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <string>
#include <roam_mapping/Octomap_multi.h>
#include <time.h>


class SemanticOctomapGeneratorNode{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in SemanticOctomapGenerator
     */
    SemanticOctomapGeneratorNode(ros::NodeHandle& nh);
    /// Desturctor
    virtual ~SemanticOctomapGeneratorNode();
    /// Reset values to paramters from parameter server
    void reset();
    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    
    void octomapCallback(const roam_mapping::Octomap_multi::ConstPtr& octomap_multi_msg);
    
    void publish2DOccupancyMap(const SemanticOctree* octomap,
                               const ros::Time& stamp,
                               const std::string& frame_id,
                               const float sensor_z = 0);
    
    static void expandNodeRecurse(SemanticOctree* inc_octomap_ptr, SemanticsOcTreeNode* node, unsigned int depth);
    
    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    bool save(const char* filename) const;
    void setWriteSemantics(bool write);

protected:
    OctomapGeneratorBase<SemanticOctree>* octomap_generator_; ///<Octomap instance pointer
    ros::NodeHandle nh_; ///<ROS handler
    ros::Publisher fullmap_pub_; ///<ROS publisher for full octomap message
    ros::Publisher colormap_pub_; ///<ROS publisher for color octomap message
    ros::Publisher occ_map_pub_; ///<ROS publisher for 2D occupancy map message
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
    tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
    ros::Subscriber octomap_sub_; ///<ROS subscriber for octomap message
    std::string world_frame_id_; ///<Id of the world frame
    std::string pointcloud_topic_; ///<Topic name for subscribed pointcloud message
    std::string octomap_topic_; ///<Topic name for subscribed octomap message
    std::string robot_name_; ///<Agent name
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
    float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
    float psi_; ///<Increment update value for a semantic class
    float phi_; ///<Decrement update value for a semantic class
    float resolution_; ///<Resolution of octomap
    float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
    float prob_hit_;  ///<Hit probability of sensor
    float prob_miss_; ///<Miss probability of sensor
    float consensus_weight; ///<Consensus weight for distributed estimation
    time_t time_last_sub; ///<Time when the last map integration occured
    time_t time_last_pub; ///<Time when the last map publication occured
    bool integration_in_progress; ///<Check if map integration is in progresss
    double sub_timer; ///<Time between each map integration
    double pub_timer; ///<Time between each map publication
    bool publish_2d_map;
    double min_ground_z;
    double max_ground_z;
    roam_mapping::Octomap_multi map_msg_; ///<ROS octomap message
};


#endif //SEMANTIC_MULTI_OCTOMAP_GENERATOR_ROS_H
