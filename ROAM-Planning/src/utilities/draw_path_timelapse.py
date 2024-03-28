#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import rospy
import tf

import numpy as np
import matplotlib.pyplot as plt
import cv2

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import OccupancyGrid

from mapping.costmap import Costmap
from utilities.util import xy_to_rc


class DrawPaths:
    def __init__(self):
        self.num_agents = rospy.get_param('~planning/multi/num_agents')
        self.horizon = rospy.get_param('~planning/multi/horizon')
    
        self.team_path_sub = rospy.Subscriber("/planner/multi/team_path", numpy_msg(Floats), self.team_path_callback, queue_size = 1)
        self.need_plan_sub = rospy.Subscriber("/planner/multi/need_plan", numpy_msg(Floats), self.need_plan_callback, queue_size = 1)
        
        self.map_sub = rospy.Subscriber("/robot_name/occupancy_map_2D", OccupancyGrid, self.map_callback) # The variable 'robot_name' should be set properly
        
        self.planning_map = None
        
        self.ready_draw = False
        self.iteration = 0
        self.all_team_path = np.zeros((self.num_agents, self.num_agents * self.horizon * 3), dtype=np.float32)
        
        self.color_list = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
        				   (127, 127, 0), (0, 127, 127), (127, 0, 127)] # Assuming self.num_agents = 6

    def map_callback(self, occ_map_msg):
        occupancy_map = 255 - np.array(occ_map_msg.data, dtype=np.uint8)
        occupancy_map = occupancy_map.reshape((occ_map_msg.info.height,
                                               occ_map_msg.info.width))
        occupancy_map = np.flipud(occupancy_map)
        
        occupancy_map[occupancy_map == 0] = Costmap.UNEXPLORED
        occupancy_map[occupancy_map == 155] = Costmap.OCCUPIED
        
        origin = self.pose_msg_to_state(occ_map_msg.info.origin)[:2]
        resolution = occ_map_msg.info.resolution

        self.planning_map = Costmap(occupancy_map, resolution, origin)
    
    def pose_msg_to_state(self, pose_msg):
        position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                               pose_msg.orientation.z, pose_msg.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return np.array([position[0], position[1], euler[2]])
    
    def team_path_callback(self, team_path_msg):
    	self.all_team_path[int(team_path_msg.data[0]) - 1, :] = team_path_msg.data[1:]
      
        if self.ready_draw:
        	vis_map = self.planning_map.copy()
        	vis_map.data = cv2.cvtColor(vis_map.data, cv2.COLOR_GRAY2RGB)
        
        	for robot_plan_ind in range(self.num_agents):
        		for robot_path_ind in range(self.num_agents):
        			path = self.all_team_path[robot_plan_ind, 3 * self.horizon * robot_path_ind:3 * self.horizon * (robot_path_ind + 1)].reshape(self.horizon, 3)
        			for pose in path:
        				pose_rc = xy_to_rc(pose[:2], vis_map).astype(int)
        				
        				cv2.circle(vis_map.data, (pose_rc[1], pose_rc[0]), 5, self.color_list[robot_plan_ind], -1)
        				
        				vis_map.data = cv2.arrowedLine(vis_map.data, (pose_rc[1], pose_rc[0]),
                                       				  (pose_rc[1] + np.round(20 * np.cos(pose[2])).astype(np.int),
                                        			  pose_rc[0] - np.round(20 * np.sin(pose[2])).astype(np.int)), self.color_list[robot_plan_ind], 2,
                                       				  tipLength=0.5)
            
        	plt.figure()
        	plt.imshow(vis_map.data)
        	plt.axis('off')
        	plt.savefig(str(self.iteration) + ".pdf", format='pdf', bbox_inches='tight')
        	
        	print("New path drawing created!")
    		
        	self.iteration += 1
        	self.ready_draw = False
    
    def need_plan_callback(self, need_plan_msg):
        self.ready_draw = np.prod(need_plan_msg.data[1:]).astype(bool)


def main(args):
    rospy.init_node('draw_paths', anonymous=True)
    compute_path_phi = DrawPaths()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Draw paths node stopped!")


if __name__ == '__main__':
    main(sys.argv)
