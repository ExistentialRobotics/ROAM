#include <octomap_multi/octomap_generator_ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/conversions.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy      

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh): nh_(nh)
{
    // Initiate octree
    octomap_generator_ = new OctomapGenerator<PCLCloud, OccupancyOctree>();

    reset();
    
    occ_octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>(robot_name + "/octomap_full", 1, true);
    occ_2d_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(robot_name + "/occupancy_map_2D", 1, true);
    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, pointcloud_topic_, 5);
    tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
    tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &OctomapGeneratorNode::octomapCallback, this);
    ROS_INFO("Occupancy octomap initialized!");
}

OctomapGeneratorNode::~OctomapGeneratorNode() {}
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{
    nh_.getParam("agent_name", robot_name);
    robot_name = "/" + robot_name;
    nh_.getParam("/octomap/pointcloud_topic", pointcloud_topic_);
    pointcloud_topic_ = robot_name + pointcloud_topic_;
    nh_.getParam("octomap/octomap_topic", octomap_topic_);
    ROS_INFO("%s\n", pointcloud_topic_.c_str());
    
    nh_.getParam("/octomap/frame_id", world_frame_id_);
    nh_.getParam("/octomap/resolution", resolution_);
    nh_.getParam("/octomap/max_range", raycast_range_);
    nh_.getParam("/octomap/min_occ_prob", clamping_thres_min_);
    nh_.getParam("/octomap/max_occ_prob", clamping_thres_max_);
    nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
    nh_.getParam("/octomap/prob_hit", prob_hit_);
    nh_.getParam("/octomap/prob_miss", prob_miss_);
    nh_.getParam("/octomap/min_ground_z", min_ground_z);
    nh_.getParam("/octomap/max_ground_z", max_ground_z);
    nh_.getParam("/octomap/consensus_weight", consensus_weight);
    nh_.getParam("/octomap/comm_timer", comm_timer);
    octomap_generator_->setClampingThresMin(clamping_thres_min_);
    octomap_generator_->setClampingThresMax(clamping_thres_max_);
    octomap_generator_->setResolution(resolution_);
    octomap_generator_->setOccupancyThres(occupancy_thres_);
    octomap_generator_->setProbHit(prob_hit_);
    octomap_generator_->setProbMiss(prob_miss_);
    octomap_generator_->setRayCastRange(raycast_range_);
    
    time_last_comm = time(0);
    integration_in_progress = false;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (!integration_in_progress)
  {
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // Get tf transform
    tf::StampedTransform sensorToWorldTf;
    try
    {
      tf_listener_.lookupTransform(world_frame_id_, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }
    // Transform coordinate
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    octomap_generator_->insertPointCloud(cloud, sensorToWorld);
    // Publish full octomap
    map_msg_.header.frame_id = world_frame_id_;
    map_msg_.header.stamp = cloud_msg->header.stamp;
    
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        occ_octomap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing full OctoMap");

    // Publish 2D occupancy map
    publish2DOccupancyMap(octomap_generator_->getOctree(), cloud_msg->header.stamp, world_frame_id_);
  }
}

void OctomapGeneratorNode::publish2DOccupancyMap(const OccupancyOctree* octomap,
                                                 const ros::Time& stamp,
                                                 const std::string& frame_id)
{
  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

  unsigned int tree_depth = octomap->getTreeDepth();

  octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);

  nav_msgs::OccupancyGrid::Ptr occupancy_map (new nav_msgs::OccupancyGrid());

  unsigned int width, height;
  double res;

  unsigned int ds_shift = tree_depth-16;

  occupancy_map->header.stamp = stamp;
  occupancy_map->header.frame_id = frame_id;
  occupancy_map->info.resolution = res = octomap->getNodeSize(16);
  occupancy_map->info.width = width = (maxX-minX) / res + 1;
  occupancy_map->info.height = height = (maxY-minY) / res + 1;
  occupancy_map->info.origin.position.x = minX  - (res / (float)(1<<ds_shift) ) + res;
  occupancy_map->info.origin.position.y = minY  - (res / (float)(1<<ds_shift) );

  occupancy_map->data.clear();
  occupancy_map->data.resize(width*height, -1);

    // traverse all leafs in the tree:
  unsigned int treeDepth = std::min<unsigned int>(16, octomap->getTreeDepth());
  for (typename OccupancyOctree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
  {
  
    double node_z = it.getZ();
    double node_half_side = pow(it.getSize(), 1/3) / 2;
    double top_side = node_z + node_half_side;
    double bottom_side = node_z - node_half_side;

    //octomap::ColorOcTreeNode::Color color = it->getSemantics().getSemanticColor();

    //if(color.b != 255)
    //    ROS_INFO("%u %u %u", color.r, color.g, color.b);
    
    if((bottom_side >= min_ground_z && bottom_side <= max_ground_z) ||
       (top_side >= min_ground_z && top_side <= max_ground_z) ||
       (bottom_side <= min_ground_z && top_side >= max_ground_z))
    {
      bool occupied = octomap->isNodeOccupied(*it);
      int intSize = 1 << (16 - it.getDepth());

      octomap::OcTreeKey minKey=it.getIndexKey();


      for (int dx = 0; dx < intSize; dx++)
      {
        for (int dy = 0; dy < intSize; dy++)
        {
          int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
          posX>>=ds_shift;

          int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
          posY>>=ds_shift;

          int idx = width * posY + posX;

          if (occupied) {
            occupancy_map->data[idx] = 100;
          }
          else if (occupancy_map->data[idx] == -1)
          {
            occupancy_map->data[idx] = 0;
          } 

        }
      }
    }
  }

  occ_2d_map_pub_.publish(*occupancy_map);
}

void OctomapGeneratorNode::octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{
  time_t now = time(0);
  double time_diff = difftime(now, time_last_comm);
  if (comm_timer < time_diff)
  {
    integration_in_progress = true;
    
    OccupancyOctree* inc_octomap_ptr = new OccupancyOctree(octomap_msg->resolution);
    if (inc_octomap_ptr){
      std::stringstream datastream;
      if (octomap_msg->data.size() > 0){
        datastream.write((const char*) &(octomap_msg->data[0]), octomap_msg->data.size());
        inc_octomap_ptr->readData(datastream);
      }
    }
    
    for(OccupancyOctree::leaf_iterator it = inc_octomap_ptr->begin_leafs(),
        end=inc_octomap_ptr->end_leafs(); it!= end; ++it)
    {
      unsigned int depth = it.getDepth();
      octomap::OcTreeNode& node = *it;
      expandNodeRecurse(inc_octomap_ptr, &node, depth);
    }

    OccupancyOctree* host_octomap_ptr = octomap_generator_->getOctree();

    for(OccupancyOctree::leaf_iterator it = inc_octomap_ptr->begin_leafs(),
        end=inc_octomap_ptr->end_leafs(); it!= end; ++it)
    {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      float logOdds = static_cast< float >(it->getValue());
      host_octomap_ptr->updateNode(x, y, z, consensus_weight * logOdds, false);
    }
    
    time_last_comm = time(0);
    integration_in_progress = false;
  }
}

void OctomapGeneratorNode::expandNodeRecurse(OccupancyOctree* inc_octomap_ptr, octomap::OcTreeNode* node, unsigned int depth)
{
  if (depth != 16)
  {
    inc_octomap_ptr->expandNode(node);
    if (depth < 15)
    {
      for (unsigned int i=0; i<8; i++) {
        octomap::OcTreeNode* child = inc_octomap_ptr->getNodeChild(node, i);
        expandNodeRecurse(inc_octomap_ptr, child, depth + 1);
      }
    }
  }
}

bool OctomapGeneratorNode::save(const char* filename) const
{
    return octomap_generator_->save(filename);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_multi");
    ros::NodeHandle nh("~");
    OctomapGeneratorNode octomapGeneratorNode(nh);
    ros::spin();
    
    std::string save_path_full;
    std::string save_path_color;
    nh.getParam("/octomap/save_path_full", save_path_full);
    nh.getParam("/octomap/save_path_color", save_path_color);

    if (!save_path_full.empty())
    {
      octomapGeneratorNode.setWriteSemantics(true);
      octomapGeneratorNode.save(save_path_full.c_str());
      ROS_INFO("Full OctoMap saved.");
    }
    
    if (!save_path_color.empty())
    {
      octomapGeneratorNode.setWriteSemantics(false);
      octomapGeneratorNode.save(save_path_color.c_str());
      ROS_INFO("Color OctoMap saved.");
    }
    
    return 0;
}
