#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import time
import rospy
import tf
import message_filters

import numpy as np

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped, Twist


class PathNavigation:

    def __init__(self):
        self.robot_name = '/' + rospy.get_param("~agent_name", 'husky')
        num_agents = rospy.get_param(self.robot_name + '/planning/multi/num_agents')
    
        tau = rospy.get_param(self.robot_name + '/navigation/tau')
        self.world_frame_id = rospy.get_param(self.robot_name + '/octomap/frame_id')
        
        dist = rospy.get_param(self.robot_name + '/navigation/dist')
        safety_dist = rospy.get_param(self.robot_name + '/navigation/safety_dist')
        
        publish_odom = rospy.get_param(self.robot_name + '/navigation/publish_odom')
        
        self.path_sub = message_filters.Subscriber(self.robot_name + '/planner/path', Path, queue_size = 15)
        self.path_sub.registerCallback(self.path_callback)
        
        self.collision_sub = message_filters.Subscriber(self.robot_name + '/planner/collision', Empty, queue_size = 15)
        self.collision_sub.registerCallback(self.collision_callback)
        
        self.cmd_vel_pub = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size = 15)
        self.path = None
        
        self.position_cmd_pub = rospy.Publisher(self.robot_name + "/planner/position_cmd", PoseStamped, queue_size = 15)
        self.odom_pub = rospy.Publisher(self.robot_name + "/planner/odom", Odometry, queue_size = 15)
        
        robot_frame_id = self.robot_name + '/base_link' # Assuming the robot frame is named as 'robot_i/base_link'
        self.tf_listener = tf.TransformListener()
        
        rospy.sleep(1)
        
        while True:
            if self.path is not None:
                next_pose = self.path[0].pose
                robot_pose = self.get_pose_from_tf(robot_frame_id)
                pose_dist = np.sqrt((robot_pose[0] - next_pose.position.x)**2 + (robot_pose[1] - next_pose.position.y)**2)
                if pose_dist > dist:
                    
                    peer_collision = False
                    for i in range(num_agents):
                        if self.robot_name[-1] == str(i + 1):
                            continue
                        peer_frame_id = self.robot_name[:-1] + str(i + 1) + '/base_link' # Assuming the robot frame is named as 'robot_i/base_link'
                        peer_pose = self.get_pose_from_tf(peer_frame_id)
                        if np.linalg.norm(np.array([next_pose.position.x - peer_pose[0], next_pose.position.y - peer_pose[1]])) <= safety_dist:
                            peer_collision = True
                            rospy.loginfo("{}: Possible close call! Stopping for deconflicting...".format(self.robot_name))
                            break
                    
                    if peer_collision:
                        self.cmd_vel_pub.publish(Twist())
                        rospy.sleep(0.5)
                        continue
                    
                    position_cmd_msg = PoseStamped()
                    position_cmd_msg.pose = next_pose
                    position_cmd_msg.header.frame_id = self.world_frame_id
                    position_cmd_msg.header.stamp = rospy.Time.now()
                    self.position_cmd_pub.publish(position_cmd_msg)
                    
                    if publish_odom:
                        odom_msg = self.get_odom_from_tf(robot_frame_id)
                        self.odom_pub.publish(odom_msg)
                else:
                    if len(self.path) > 1:
                        self.path = self.path[1:]
                    else:
                        self.path = None
            else:
                self.cmd_vel_pub.publish(Twist())
            rospy.sleep(tau)
                
    def get_pose_from_tf(self, from_frame_id):
        (translation, rotation) = self.get_tf(from_frame_id)
        
        euler = tf.transformations.euler_from_quaternion(rotation)
        return np.array([translation[0], translation[1], euler[2]])

    def get_odom_from_tf(self, from_frame_id):
        (translation, rotation) = self.get_tf(from_frame_id)
        
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = translation[2]
        odom_msg.pose.pose.orientation.x = rotation[0]
        odom_msg.pose.pose.orientation.y = rotation[1]
        odom_msg.pose.pose.orientation.z = rotation[2]
        odom_msg.pose.pose.orientation.w = rotation[3]
        odom_msg.header.frame_id = self.world_frame_id
        odom_msg.child_frame_id = from_frame_id
        odom_msg.header.stamp = rospy.Time.now()
        return odom_msg

    def get_tf(self, from_frame_id):
        no_transfrom = True
        while no_transfrom:
            try:
                (translation, rotation) = self.tf_listener.lookupTransform(self.world_frame_id,
                                                                           from_frame_id,
                                                                           rospy.Time(0))
                no_transfrom = False
            except tf.LookupException:
                continue
        return (translation, rotation)
    
    def path_callback(self, path_msg):
        rospy.loginfo("{}: Received a path from exploration algorithm!".format(self.robot_name))
        self.path = path_msg.poses
        
    def collision_callback(self, collision_msg):
        self.path = None


def main(args):
    rospy.init_node('path_navigation', anonymous=True)
    path_navigation = PathNavigation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Navigation stopped!")


if __name__ == '__main__':
    main(sys.argv)
