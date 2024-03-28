#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class PclGenerator:
    def __init__(self, intrinsic, width = 80, height = 60, frame_id = "husky/camera"):
        '''
        width: (int) width of input image
        height: (int) height of input image
        '''
        self.intrinsic = intrinsic
        # Allocate arrays
        x_index = np.array([list(range(width))*height], dtype = '<f4')
        y_index = np.array([[i]*width for i in range(height)], dtype = '<f4').ravel()
        self.xy_index = np.vstack((x_index, y_index)).T # x,y
        self.xyd_vect = np.zeros([width*height, 3], dtype = '<f4') # x,y,depth
        self.XYZ_vect = np.zeros([width*height, 3], dtype = '<f4') # real world coord
        self.ros_data = np.ones([width*height, 3], dtype = '<f4') # [x,y,z]
        # Prepare ros cloud msg
        # Cloud data is serialized into a contiguous buffer, set fields to specify offsets in buffer
        self.cloud_ros = PointCloud2()
        self.cloud_ros.header.frame_id = frame_id
        self.cloud_ros.height = 1
        self.cloud_ros.width = width*height
        self.cloud_ros.fields.append(PointField(
            name = "x",
            offset = 0,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "y",
            offset = 4,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "z",
            offset = 8,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.is_bigendian = False
        self.cloud_ros.point_step = 3 * 4 # In bytes
        self.cloud_ros.row_step = self.cloud_ros.point_step * self.cloud_ros.width * self.cloud_ros.height
        self.cloud_ros.is_dense = False

    def generate_cloud_data_common(self, depth_img):
        """
        Do depth registration
        \param depth_img (numpy array float32 2d)
        [x, y, Z] = [X, Y, Z] * intrinsic.T
        """
        np.place(depth_img, depth_img == 0, 100000) # Handle maximum range measurements
        
        depth_img = depth_img.view('<f4')
        # Add depth information
        self.xyd_vect[:,0:2] = self.xy_index * depth_img.reshape(-1,1) / 1000 # Division by 1000 for unit conversion
        self.xyd_vect[:,2:3] = depth_img.reshape(-1,1) / 1000 # Division by 1000 for unit conversion
        self.XYZ_vect = self.xyd_vect.dot(self.intrinsic.I.T)
        # Convert to ROS point cloud message in a vectorialized manner
        # ros msg data: [x,y,z] (little endian float32)
        # Concatenate data
        self.ros_data[:,0:3] = self.XYZ_vect

    def make_ros_cloud(self, stamp):
        # Assign data to ros msg
        # We should send directly in bytes, send in as a list is too slow, numpy tobytes is too slow, takes 0.3s.
        self.cloud_ros.data = self.ros_data.ravel().tobytes()
        self.cloud_ros.header.stamp = stamp
        return self.cloud_ros

    def generate_cloud(self, depth_img, stamp):
        self.generate_cloud_data_common(depth_img)
        return self.make_ros_cloud(stamp)
