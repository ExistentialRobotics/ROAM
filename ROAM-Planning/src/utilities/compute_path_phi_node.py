#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import rospy

import numpy as np

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from rospy_tutorials.msg import Floats

from utilities.util import wrap_angles


class ComputePathPhi:
    def __init__(self):
        self.theta_coeff = rospy.get_param('~planning/theta_coeff')
        self.num_agents = rospy.get_param('~planning/multi/num_agents')
        self.horizon = rospy.get_param('~planning/multi/horizon')
   
        self.team_path_sub = rospy.Subscriber("/planner/multi/team_path", numpy_msg(Floats), self.team_path_callback, queue_size = 1)
        self.need_plan_sub = rospy.Subscriber("/planner/multi/need_plan", numpy_msg(Floats), self.need_plan_callback, queue_size = 1)
        
        self.phi_pub = rospy.Publisher("path_phi", Float32, queue_size = 1)
        
        self.ready_compute_phi = False
        self.all_team_path = np.zeros((self.num_agents, self.num_agents * self.horizon * 3), dtype=np.float32)

    def team_path_callback(self, team_path_msg):
    	self.all_team_path[int(team_path_msg.data[0]) - 1, :] = team_path_msg.data[1:]
      
        if self.ready_compute_phi:
            
            phi = 0
            for i in range(self.num_agents):
            	for j in range(i + 1, self.num_agents):
            		diff = self.all_team_path[i, :] - self.all_team_path[j, :]
            		diff[2::3] = wrap_angles(diff[2::3])
            		diff[2::3] *= self.theta_coeff
            		phi += np.sum(diff**2)
            phi_msg = Float32()
            phi_msg.data = phi
            self.phi_pub.publish(phi_msg)
            
            self.ready_compute_phi = False
    
    def need_plan_callback(self, need_plan_msg):
        self.ready_compute_phi = np.prod(need_plan_msg.data[1:]).astype(bool)


def main(args):
    rospy.init_node('compute_path_phi', anonymous=True)
    compute_path_phi = ComputePathPhi()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Phi computation stopped!")


if __name__ == '__main__':
    main(sys.argv)
