#ifndef OCTOMAP_GENERATOR_ROS_H
#define OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_multi/octomap_generator.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <string>
#include <octomap_msgs/Octomap.h>
#include <time.h>

class OctomapGeneratorNode{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGeneratorNode(ros::NodeHandle& nh);
    
    /// Desturctor
    virtual ~OctomapGeneratorNode();
    
    /// Reset values to paramters from parameter server
    void reset();
    
    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
    
    void publish2DOccupancyMap(const OccupancyOctree* octomap,
                               const ros::Time& stamp,
                               const std::string& frame_id);
    
    static void expandNodeRecurse(OccupancyOctree* inc_octomap_ptr, octomap::OcTreeNode* node, unsigned int depth);
    
    /**
     * \brief Save octomap to a file.
     * \param filename The output filename
     */
    bool save(const char* filename) const;

protected:
    OctomapGeneratorBase<OccupancyOctree>* octomap_generator_; ///<Octomap instance pointer
    ros::NodeHandle nh_; ///<ROS handler
    ros::Publisher occ_octomap_pub_; ///<ROS publisher for occupancy octomap message
    ros::Publisher occ_2d_map_pub_; ///<ROS publisher for 2D occupancy map message
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
    ros::Subscriber octomap_sub_; ///<ROS subscriber for octomap message
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
    tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
    std::string world_frame_id_; ///<Id of the world frame
    std::string pointcloud_topic_; ///<Topic name for subscribed pointcloud message
    std::string octomap_topic_; ///<Topic name for subscribed octomap message
    std::string robot_name; ///<Agent name
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
    float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
    float resolution_; ///<Resolution of octomap
    float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
    float prob_hit_;  ///<Hit probability of sensor
    float prob_miss_; ///<Miss probability of sensor
    float consensus_weight; ///<Consensus weight for distributed estimation
    time_t time_last_comm; ///<Time when the last map communication occured
    bool integration_in_progress; ///<Check if map integration is in progresss
    double comm_timer; ///<Time between each map communication
    double min_ground_z;
    double max_ground_z;
    octomap_msgs::Octomap map_msg_; ///<ROS octomap message
};

#endif //OCTOMAP_GENERATOR_ROS_H
