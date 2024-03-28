#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import rospy
import message_filters
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from depth_sensor import PclGenerator


class DepthToPC:
    """
    Class for ros node to take in a depth image
    Then produce point cloud based on depth information
    """
    def __init__(self):
        robot_name = '/' + rospy.get_param("~agent_name", 'husky')
        
        # Get image size
        self.img_width, self.img_height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        # Set up ROS
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        # Point cloud frame id
        frame_id = robot_name + rospy.get_param('/depth_image/frame_id')
        # Camera intrinsic matrix
        fx = rospy.get_param('/camera/fx')
        fy = rospy.get_param('/camera/fy')
        cx = rospy.get_param('/camera/cx')
        cy = rospy.get_param('/camera/cy')
        intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
        
        # Noise configuration
        self.depth_noise_std = rospy.get_param('/depth_image/depth_noise_std')
        self.noisy_obs = False
        if self.depth_noise_std > 0:
            self.noisy_obs = True
        
        self.pcl_pub = rospy.Publisher(robot_name + rospy.get_param('/octomap/pointcloud_topic'), PointCloud2, queue_size = 1)

        # increase buffer size to avoid delay (despite queue_size = 1)
        self.depth_sub = message_filters.Subscriber(robot_name + rospy.get_param('/depth_image/depth_image_topic'), Image,
                                                    queue_size = 1, buff_size = 30*self.img_width*self.img_height)
                                                    
        # Take in depth image
        self.depth_sub.registerCallback(self.depth_callback)
        self.cloud_generator = PclGenerator(intrinsic, self.img_width, self.img_height, frame_id)
        print('Point cloud ready!')

    def depth_callback(self, depth_img_ros):
        """
        Callback function to produce point cloud based on input depth image
        """
        # Convert ros Image message to numpy array
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect',
                               anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)
        
        # Add noise
        if self.noisy_obs is True:
            depth_img = self.add_noise(depth_img)
        
        cloud_ros = self.cloud_generator.generate_cloud(depth_img, depth_img_ros.header.stamp)

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)

    def add_noise(self, depth_img):
        noisy_depth_img = depth_img + np.random.normal(0, self.depth_noise_std, depth_img.shape).astype(np.float32)
        np.place(noisy_depth_img, depth_img == 0, 0)
        return noisy_depth_img
        

def main(args):
    rospy.init_node('depth_to_point_cloud', anonymous=True)
    depth_to_point_cloud = DepthToPC()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Depth to point cloud conversion shutting down!")


if __name__ == '__main__':
    main(sys.argv)
