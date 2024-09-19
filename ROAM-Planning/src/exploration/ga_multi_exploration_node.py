#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import time
import rospy
import tf
import cv2

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import os
from utilities.frontier_utils import extract_frontiers, cleanup_map_for_planning
from footprints.footprint_points import get_tricky_circular_footprint, get_tricky_oval_footprint, get_jackal_footprint
from footprints.footprints import CustomFootprint
from planners.astar_cpp import oriented_astar, get_astar_angles
from utilities.util import rc_to_xy, wrap_angles, xy_to_rc
from mapping.costmap import Costmap
from utilities.sensor_utils import bresenham2d, bresenham2d_with_intensities

def map_visulize(image_path, res = 1, origin = np.array([0, 0]), goal = None, radius = None):
    if isinstance(image_path, str):
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f'Image file not found: {image_path}')
    elif isinstance(image_path, np.ndarray):
        img = image_path
    else:
        raise ValueError('Invalid image path')
    img_height, img_width = img.shape
    img = np.flipud(img)
    map_width = img_width * res
    map_height = img_height * res
    fig, ax = plt.subplots()
    extent = [origin[0], origin[0] + map_width, origin[1], origin[1] + map_height]
    ax.imshow(img, cmap='gray', extent=extent, origin='lower')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Map')
    if goal is not None and radius is not None:
        ax.add_patch(plt.Circle(goal, radius, color='r', alpha=0.3))
    # plt.show()
    return ax

class GradAscentMultiExplorationAgent:
    def __init__(self):
        self.robot_name = '/' + rospy.get_param("~agent_name", 'husky')
        
        footprint_type = rospy.get_param(self.robot_name + '/footprint/type')
        if not os.path.exists('/root/ws/map'):
            os.makedirs('/root/ws/map')
        files = os.listdir('/root/ws/map')
        for f in files:
            os.remove('/root/ws/map/' + f)    
        self.filter_obstacles_flag = False
        self.map_counter = 0
        if footprint_type == 'tricky_circle':
            footprint_points = get_tricky_circular_footprint()
        elif footprint_type == 'tricky_oval':
            footprint_points = get_tricky_oval_footprint()
        elif footprint_type == 'circle':
            rotation_angles = np.arange(0, 2 * np.pi, 4 * np.pi / 180)
            footprint_radius = rospy.get_param(self.robot_name + '/footprint/radius')
            footprint_points = \
                footprint_radius * np.array([np.cos(rotation_angles), np.sin(rotation_angles)]).T
        elif footprint_type == 'pixel':
            footprint_points = np.array([[0., 0.]])
        elif footprint_type == 'jackal':
            footprint_points = get_jackal_footprint()
        else:
            footprint_points = None
            assert False and "footprint type specified not supported."

        self.footprint = CustomFootprint(footprint_points=footprint_points,
                                          angular_resolution=np.pi / 4,
                                          inflation_scale=rospy.get_param(self.robot_name + '/footprint/inflation_scale'))
        
        self.footprint_mask_radius = None

        self.planning_angles = get_astar_angles()

        self.min_frontier_size = rospy.get_param(self.robot_name + '/planning/min_frontier_size')

        self.angular_range = rospy.get_param(self.robot_name + '/planning/angular_range')
        self.angular_samples = rospy.get_param(self.robot_name + '/planning/angular_samples')
        
        num_orientation_samples = rospy.get_param(self.robot_name + '/planning/num_orientation_samples')
        self.precomputed_ray_inds = self.precompute_ray_inds(num_orientation_samples)
        self.precomputed_orientation_angles = np.linspace(0, 2 * np.pi, num_orientation_samples, endpoint=False)

        self.distance_coeff = rospy.get_param(self.robot_name + '/planning/distance_coeff')

        self.theta_coeff = rospy.get_param(self.robot_name + '/planning/theta_coeff')

        self.SE2_radius = rospy.get_param(self.robot_name + '/planning/SE2_radius')

        self.max_iter = rospy.get_param(self.robot_name + '/planning/max_iter')
        self.adjusted_step_size = rospy.get_param(self.robot_name + '/planning/step_size') / self.SE2_radius
        
        self.max_path_length = rospy.get_param(self.robot_name + '/planning/max_path_length')

        self.raytrace_precomputed = None
        self.raytrace_mask = None

        self.f_precomputed = None
        self.min_log_odd = None
        self.max_log_odd = None
        
        self.max_occ_prob = rospy.get_param(self.robot_name + '/octomap/clamping_thres_max')
        self.min_occ_prob = rospy.get_param(self.robot_name + '/octomap/clamping_thres_min')
        max_log_odd = np.log(self.max_occ_prob / (1 - self.max_occ_prob))
        min_log_odd = np.log(self.min_occ_prob / (1 - self.min_occ_prob))
        prob_hit = rospy.get_param(self.robot_name + '/octomap/prob_hit')
        delta = np.log(prob_hit / (1 - prob_hit))
        self.precompute_f(max_log_odd, min_log_odd, delta)
        
        self.path_pub = rospy.Publisher(self.robot_name + "/planner/path", Path, queue_size = 15)
        
        self.collision_pub = rospy.Publisher(self.robot_name + "/planner/collision", Empty, queue_size = 15)
        
        self.tf_listener = tf.TransformListener()
        
        self.world_frame_id = rospy.get_param(self.robot_name + '/octomap/frame_id')
        self.robot_frame_id = self.robot_name + '/base_link'
        
        self.is_planning = False
        self.curr_path = None
        self.waypoint_inds = None
        self.goal = None
        self.goal_check_radius = rospy.get_param(self.robot_name + '/planning/goal_check_radius')
        self.draw_radius = rospy.get_param(self.robot_name + '/planning/draw_radius')
        
        self.map_sub = rospy.Subscriber(rospy.get_param(self.robot_name + '/planning/map_topic'), OccupancyGrid, self.map_callback)
        self.collision_check_timer = rospy.get_time()
        self.collision_check_period = rospy.get_param(self.robot_name + '/planning/collision_check_period')
        
        # Multi-agent components
        self.num_agents = rospy.get_param(self.robot_name + '/planning/multi/num_agents')
        self.horizon = rospy.get_param(self.robot_name + '/planning/multi/horizon')
        self.agent_id = int(self.robot_name[-1]) # Assuming robots are named as 'robot_1', 'robot_2', ...

        self.team_path_sub = rospy.Subscriber("/planner/multi/team_path", numpy_msg(Floats), self.team_path_callback, queue_size = 15)     
        self.team_path_pub = rospy.Publisher("/planner/multi/team_path", numpy_msg(Floats), queue_size = 15)
        
        self.delta_q = 2 * rospy.get_param(self.robot_name + '/planning/multi/delta_q')
        self.gamma_q = 2 * rospy.get_param(self.robot_name + '/planning/multi/gamma_q')
        self.robot_colision_coeff = 2 * rospy.get_param(self.robot_name + '/planning/multi/robot_colision_coeff')
        
        self.consensus_delta = None
        self.received_team_path_num = 0
        consensus_epsilon = rospy.get_param(self.robot_name + '/planning/multi/epsilon')
        self.iter = 0
        
        self.team_path = np.zeros(1 + 3 * self.horizon * self.num_agents, dtype=np.float32)
        self.team_path[0] = self.agent_id
        
        self.weight_mask = np.ones(3 * self.horizon * self.num_agents)
        self.weight_mask[2::3] = self.theta_coeff
        self.weight_mask *= consensus_epsilon
        
        self.need_plan = np.zeros(1 + self.num_agents, dtype=np.float32)
        self.need_plan[0] = self.agent_id
        self.need_plan_sub = rospy.Subscriber("/planner/multi/need_plan", numpy_msg(Floats), self.need_plan_callback, queue_size = 15)     
        self.need_plan_pub = rospy.Publisher("/planner/multi/need_plan", numpy_msg(Floats), queue_size = 15)
        self.should_plan_thresh = rospy.get_param(self.robot_name + '/planning/multi/should_plan_threshold')
        self.need_plan_thresh = rospy.get_param(self.robot_name + '/planning/multi/need_plan_threshold')
        self.need_plan_timer_period = rospy.get_param(self.robot_name + '/planning/multi/need_plan_period')
        self.need_plan_publish_timer = rospy.Timer(rospy.Duration(self.need_plan_timer_period), self.need_plan_publish)
        
        self.planning_map = None
        self.distance_map = None
        self.probability_map = None
        self.log_odds_map_data = None
        self.vectorized_rc = None
        self.vectorized_xy = None
        self.eval_cells = None
        self.kernel_cleanup = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.kernel_erosion = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # self.kernel_cleanup = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # self.kernel_erosion = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.plan_map_pub1 = rospy.Publisher(self.robot_name + "/PLANMAP1", OccupancyGrid, queue_size = 1, latch=True)
        self.plan_map_pub = rospy.Publisher(self.robot_name + "/PLANMAP", OccupancyGrid, queue_size = 1, latch=True) 
        rospy.sleep(1)
        print('Multi agent exploration agent \'{}\' initialized!'.format(self.robot_name))

    def team_path_callback(self, team_path_msg):
        
        if team_path_msg.data[0] == self.agent_id:
            return
        
        if self.received_team_path_num == 0:
            self.consensus_delta = np.copy(team_path_msg.data[1:])
        else:
            self.consensus_delta += team_path_msg.data[1:]
        
        self.received_team_path_num += 1
    
    def need_plan_callback(self, need_plan_msg):
        
        self.need_plan[1:] = need_plan_msg.data[1:]

        if self.check_ready():
            self.need_plan[self.agent_id] = 1
            
            if np.mean(self.need_plan[1:]) >= self.should_plan_thresh:
                self.optimize_pose()
        else:
            if self.is_planning:
                self.need_plan[self.agent_id] = 1
            else:
                self.need_plan[self.agent_id] = 0

    def optimize_pose(self):
        if self.is_planning:
            return
        
        # print('IN OPTIMIZE_POSE Planning started for agent \'{}\'!'.format(self.robot_name))
        self.is_planning = True
        self.consensus_delta = None
        self.received_team_path_num = 0
        self.make_init_plan()
        self.team_path_pub.publish(self.team_path)

        for iter_num in range(self.max_iter):
            print('{}: {}-th iteration of pose optimization started! received paths = {}'.format(self.robot_name, iter_num + 1, self.received_team_path_num))
            if np.mean(self.need_plan[1:]) < self.need_plan_thresh:
                print('{}: Peer-pressured to stop planning!'.format(self.robot_name))
                break
        
            if self.received_team_path_num > 0:
                unweighted_delta = self.consensus_delta - self.received_team_path_num * self.team_path[1:]
                self.team_path[1:] += self.weight_mask * unweighted_delta / (self.received_team_path_num + 1)
                self.team_path[3::3] = wrap_angles(self.team_path[3::3])
                print('{}: consensus step norm of team path change = {}'.format(self.robot_name, np.linalg.norm(self.weight_mask * unweighted_delta / (self.received_team_path_num + 1))))
                self.consensus_delta = None
                self.received_team_path_num = 0

            team_grad = np.zeros_like(self.team_path[1:])
            for i in range(self.horizon):
                pose = self.team_path[1 + 3 * self.horizon * (self.agent_id - 1) + 3 * i:1 + 3 * self.horizon * (self.agent_id - 1) + 3 * (i + 1)]
                cell_indx, nearest_indx = self.get_neighborhood(pose)
                reachable_rc = self.check_reachability(cell_indx, nearest_indx)
                cell_scores = self.compute_score(reachable_rc)
                team_grad[3 * self.horizon * (self.agent_id - 1) + 3 * i:\
                          3 * self.horizon * (self.agent_id - 1) + 3 * (i + 1)] += self.compute_gradient(pose, cell_scores, reachable_rc)
                
                for agent in range(self.num_agents):
                    t_init = 0
                    robot_collision_penalty = self.robot_colision_coeff
                    if agent + 1 == self.agent_id:
                        t_init = i + 1
                        robot_collision_penalty = 1
                    
                    for t in range(t_init, self.horizon):
                        other_pose = self.team_path[1 + 3 * self.horizon * agent + 3 * t:1 + 3 * self.horizon * agent + 3 * (t + 1)]
                        dist = np.linalg.norm(pose[:2] - other_pose[:2])
                        if dist <= self.delta_q:
                            noise = np.zeros(2)
                            if dist < 1e-6:
                                noise = np.random.normal(size=2) * 1e-3
                            grad = robot_collision_penalty * self.gamma_q * (1 - self.delta_q / (dist + np.linalg.norm(noise))) * (pose[:2] - other_pose[:2] + noise)
                            team_grad[3 * self.horizon * (self.agent_id - 1) + 3 * i:\
                                      3 * self.horizon * (self.agent_id - 1) + 3 * i + 2] -= grad
                            team_grad[3 * self.horizon * agent + 3 * t:\
                                      3 * self.horizon * agent + 3 * t + 2] += grad
            
            self.team_path[1:] += team_grad * self.adjusted_step_size / (iter_num + 1)
            self.team_path[3::3] = wrap_angles(self.team_path[3::3])
            
            self.team_path_pub.publish(self.team_path)
            print('{}: {}-th iteration of pose optimization Finished! Gradient norm: {}'.format(self.robot_name, iter_num + 1, np.linalg.norm(team_grad * self.adjusted_step_size / (iter_num + 1))))
        
        self.publish_path()
        self.is_planning = False
    
    def make_init_plan(self):
        print('{}: LOOKING FOR FRONTIERS!'.format(self.robot_name))
        if self.raytrace_precomputed is None:
            print('{}: Initializing ray-tracing!'.format(self.robot_name))
            max_range = rospy.get_param(self.robot_name + '/octomap/max_range')
            self.precompute_raytracing(max_range)
            self.footprint_mask_radius = self.footprint.get_mask_radius(self.planning_map.resolution)
        
        map_shape = self.planning_map.get_shape()
        self.vectorized_rc = np.mgrid[0:map_shape[0], 0:map_shape[1]].reshape((2, map_shape[0] * map_shape[1])).astype(int).T
        self.vectorized_xy = rc_to_xy(self.vectorized_rc, self.planning_map)
        
        free_mask = np.zeros_like(self.planning_map.data, dtype=np.uint8)
        free_mask[self.planning_map.data == Costmap.FREE] = 1
        
        self.distance_map = cv2.distanceTransform(free_mask, cv2.DIST_L2,
                                                  cv2.DIST_MASK_PRECISE) * self.planning_map.resolution
        
        self.probability_map = self.planning_map.copy()
        self.probability_map.data = np.ones_like(self.probability_map.data, dtype=float) * 0.5
        self.probability_map.data[self.planning_map.data == Costmap.OCCUPIED] = self.max_occ_prob
        self.probability_map.data[self.planning_map.data == Costmap.FREE] = self.min_occ_prob
        
        self.log_odds_map_data = np.zeros_like(self.planning_map.data, dtype=int)
        self.log_odds_map_data[self.planning_map.data == Costmap.OCCUPIED] = self.max_log_odd
        self.log_odds_map_data[self.planning_map.data == Costmap.FREE] = self.min_log_odd
        
        self.eval_cells = dict()
        
        for i in range(self.num_agents):
            frame_id = self.robot_name[:-1] + str(i + 1) + '/base_link' # Assuming the robot frame is named as 'robot_i/base_link'
            robot_pose = self.get_pose_from_tf(frame_id)

            while True:
                if self.planning_map.data.dtype == np.uint8:
                    break
                print('{}: Invalid map data type!'.format(self.robot_name))
                rospy.sleep(0.1)

            self.planning_map.data = np.array(self.planning_map.data)
            # self.footprint.draw_circumscribed(robot_pose, self.planning_map, self.draw_radius)
            path = self.compute_frontier_path(robot_pose)
            
            if path.shape[0] >= self.horizon:
                # print('length of path before sampling: ', path.shape[0])
                path_inds = np.linspace(0, path.shape[0], self.horizon, endpoint=False).astype(int)
                path_sampled = path[path_inds]
            else:
                # print('length of path: ', path.shape[0])
                path_sampled = np.zeros((self.horizon, 3))
                path_sampled[:path.shape[0]] = path
                path_sampled[path.shape[0]:] = path[-1]
                
            self.team_path[1 + 3 * self.horizon * i:1 + 3 * self.horizon * (i + 1)] = path_sampled.reshape(1, 3 * path_sampled.shape[0])

    def compute_frontier_path(self, robot_pose):
        path_list = []
        path_score_list = []
        frontiers, frontier_size = self.find_frontiers(robot_pose)
        
        robot_pose_forward = robot_pose.copy()
        robot_pose_forward[0] = robot_pose[0] + 1 * np.cos(robot_pose[2])
        robot_pose_forward[1] = robot_pose[1] + 1 * np.sin(robot_pose[2])

        # print("%d frontiers found!" % len(frontiers))
        if frontiers is not None:
            for i, f in enumerate(frontiers):
                if np.linalg.norm(robot_pose_forward[:2] - f[:2]) < 1:
                    continue
                # print("FINDING PATH TO FRONTIER: ", f)
                # why do we need to set 127 as obstacles
                plan_success, path = oriented_astar(start=robot_pose_forward,
                                                    occupancy_map=self.planning_map,
                                                    footprint=self.footprint,
                                                    obstacle_values=[Costmap.OCCUPIED, Costmap.UNEXPLORED],
                                                    epsilon=1,
                                                    goal=f)
                if plan_success and path.shape[0] > 1:
                    path_length = np.sum(np.linalg.norm(path[1:, :2] - path[:-1, :2], axis=1))
                    if path_length > self.goal_check_radius:
                        pixel_path = xy_to_rc(path, self.planning_map)
                        try:
                                collision_inds = np.nonzero(self.planning_map.data[pixel_path[:, 0].astype(int),
                                pixel_path[:, 1].astype(int)] == Costmap.OCCUPIED)[0]
                                if collision_inds.shape[0] == 0:
                                        path_list.append(path)
                                        path_score_list.append(frontier_size[i] / (1 + path_length))
                        except IndexError:
                                continue
        
        best_path = None
        if len(path_list) == 0:
            print('Planning failed! Planning a failsafe path...')
            best_path = self.get_failsafe_path(robot_pose)
        else:
            best_path_ind = np.argmax(np.array(path_score_list))
            best_path = path_list[best_path_ind]
        
        return best_path
    
    def need_plan_publish(self, event):
        self.need_plan_pub.publish(self.need_plan)

    def precompute_ray_inds(self, num_orientation_samples):
        num_inds = np.round(self.angular_range * self.angular_samples / (2 * np.pi) + 1)
        increment_step = self.angular_samples / num_orientation_samples
        ray_inds = (np.arange(num_inds) - (num_inds - 1) / 2 +
                    np.arange(num_orientation_samples)[:,None] * increment_step).astype(int) % self.angular_samples

        return ray_inds

    def precompute_raytracing(self, max_range):
        sampled_angles = np.linspace(0, 2 * np.pi, self.angular_samples, endpoint=False)
        endpoints = max_range * np.array([np.cos(sampled_angles), np.sin(sampled_angles)]).T
        endpoints_rc = xy_to_rc(endpoints, self.planning_map)
        startpoint_rc = xy_to_rc(np.array([0, 0]), self.planning_map)
        raytrace_list = []
        largest_size = 0
        for endpoint_rc in endpoints_rc:
            raytrace = bresenham2d(startpoint_rc, endpoint_rc)[1:]
            raytrace_list.append(raytrace)
            if raytrace.shape[0] > largest_size:
                largest_size = raytrace.shape[0]

        self.raytrace_precomputed = np.zeros((self.angular_samples, largest_size, 2), dtype=int)
        self.raytrace_mask = np.zeros((self.angular_samples, largest_size), dtype=bool)
        for i, raytrace in enumerate(raytrace_list):
            if raytrace.shape[0] == largest_size:
                self.raytrace_precomputed[i] = raytrace
                self.raytrace_mask[i, :] = 1
            else:
                self.raytrace_precomputed[i] = np.vstack((raytrace, np.zeros((largest_size - raytrace.shape[0], 2))))
                self.raytrace_mask[i, :raytrace.shape[0]] = 1

        self.raytrace_precomputed = (self.raytrace_precomputed - startpoint_rc).astype(int)
    
    def precompute_f(self, max_log_odd, min_log_odd, delta):
        assert np.abs(min_log_odd - np.round(min_log_odd)) < 0.01, "Minimum log-odds limit should be an integer!"
        assert np.abs(max_log_odd - np.round(max_log_odd)) < 0.01, "Maximum log-odds limit should be an integer!"
        assert np.abs(delta - 1) < 0.01, "Log-odds increment should be 1!"
        
        min_log_odd = np.round(min_log_odd).astype(int)
        max_log_odd = np.round(max_log_odd).astype(int)
        delta = 1
        
        delta_lambda = np.array([[- delta], [delta]])
        cell_lambda = np.arange(min_log_odd, max_log_odd + delta, delta)
        delta_lambda_p_lambda = cell_lambda + delta_lambda
        exp_delta_p_lambda = np.exp(delta_lambda_p_lambda)
        self.f_precomputed = np.log((1 + np.exp(cell_lambda)) / (1 + exp_delta_p_lambda)) +\
                              delta_lambda * exp_delta_p_lambda / (1 + exp_delta_p_lambda)
        self.min_log_odd = min_log_odd
        self.max_log_odd = max_log_odd

    def pose_msg_to_state(self, pose_msg):
        position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                               pose_msg.orientation.z, pose_msg.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return np.array([position[0], position[1], euler[2]])
        
    def get_pose_from_tf(self, from_frame_id):
        no_transfrom = True
        while no_transfrom:
            try:
                (translation, rotation) = self.tf_listener.lookupTransform(self.world_frame_id,
                                                                           from_frame_id,
                                                                           rospy.Time(0))
                no_transfrom = False
            except tf.LookupException or tf.ConnectivityException:
                continue
        
        euler = tf.transformations.euler_from_quaternion(rotation)        
        return np.array([translation[0], translation[1], euler[2]])

    def check_ready(self):
        '''
        return true only when robot has map,
        not planning,
        haven't reach current goal
        '''
        if self.planning_map is None:
            return False
    
        if self.is_planning:
            return False
        else:
            if self.goal is None:
                return True
            else:
                robot_pose = self.get_pose_from_tf(self.robot_frame_id)
                if np.linalg.norm(robot_pose[:2] - self.goal[:2]) < self.goal_check_radius:
                    self.goal = None
                    return True
                
                return False

    def map_callback(self, occ_map_msg):
        '''
        using new map to check if planned path has collision, if does,
        find the nearest collision point and find new path to waypoints after it
        do this check only when we have path generated before or from other func and currently not planning
        '''
        # print('{}: New map received! MAP CALLBACK START'.format(self.robot_name))
        if not self.is_planning:
            plan_map_msg = OccupancyGrid()
            plan_map_msg.header = occ_map_msg.header
            plan_map_msg.info = occ_map_msg.info
            
            occupancy_map = 255 - np.array(occ_map_msg.data, dtype=np.uint8)
            occupancy_map = occupancy_map.reshape((occ_map_msg.info.height,
                                                   occ_map_msg.info.width))
            occupancy_map = np.flipud(occupancy_map)
            
            occupancy_map[occupancy_map == 0] = Costmap.UNEXPLORED
            occupancy_map[occupancy_map == 155] = Costmap.OCCUPIED
            
            origin = self.pose_msg_to_state(occ_map_msg.info.origin)[:2]
            resolution = occ_map_msg.info.resolution

            self.planning_map = Costmap(occupancy_map, resolution, origin)
            # CHECK HERE 
            self.planning_map = cleanup_map_for_planning(occupancy_map=self.planning_map, kernel=self.kernel_cleanup, filter_obstacles=self.filter_obstacles_flag)
            self.planning_map.data = cv2.erode(self.planning_map.data, self.kernel_erosion, iterations=1)
            tmp_occ_map = self.planning_map.data.copy()
            # map_visulize(tmp_occ_map, res=self.planning_map.resolution, origin=self.planning_map.origin)
            # plt.savefig('/root/ws/map/map%d.png'%self.map_counter)
            # plt.clf()
            self.map_counter += 1
            tmp_occ_map_int = np.zeros_like(tmp_occ_map, dtype=np.int8) 
            tmp_occ_map_int[tmp_occ_map == Costmap.UNEXPLORED] = -1
            tmp_occ_map_int[tmp_occ_map == Costmap.FREE] = 0    
            tmp_occ_map_int[tmp_occ_map == Costmap.OCCUPIED] = 100
            tmp_occ_map_int = np.flipud(tmp_occ_map_int)
            plan_map_msg.data = tmp_occ_map_int.flatten().tolist()
            self.plan_map_msg = plan_map_msg
            self.plan_map_pub.publish(plan_map_msg)
            check_collision = False
            curr_time = rospy.get_time()
            if (curr_time - self.collision_check_timer) > self.collision_check_period:
                self.collision_check_timer = rospy.get_time()
                check_collision = True
            
            if self.curr_path is not None and check_collision:
                # print("Start checking collision!")
                pixel_path = xy_to_rc(self.curr_path, self.planning_map)
                try:
                    # find out the index of collision of current path
                    collision_inds = np.nonzero(self.planning_map.data[pixel_path[:, 0].astype(int),pixel_path[:, 1].astype(int)] == Costmap.OCCUPIED)[0]
                except IndexError:
                    return
                
                if collision_inds.shape[0] > 0:
                    # print("collision_inds: ", collision_inds)
                    # print('{}: Collision ahead!'.format(self.robot_name))
                    self.collision_pub.publish(Empty())
                    
                    self.is_planning = True
                    
                    # print("waypoint_inds: ", self.waypoint_inds)
                    rem_waypoint_inds = self.waypoint_inds[self.waypoint_inds > collision_inds[0]]
                    rem_waypoints = self.curr_path[rem_waypoint_inds, :]
                    
                    robot_pose = self.get_pose_from_tf(self.robot_frame_id)
                    self.footprint.draw_circumscribed(robot_pose, self.planning_map, self.draw_radius)
                    full_path = robot_pose[None, :]
                    self.waypoint_inds = np.array([0], dtype=int)
                    # print("rem_waypoints: ", rem_waypoint_inds)
                    for pose in rem_waypoints:
                        # print("Replan to waypoint: ", pose)
                        plan_success, partial_path = oriented_astar(start=full_path[-1], occupancy_map=self.planning_map, footprint=self.footprint, 
                                                                    obstacle_values=[Costmap.OCCUPIED], epsilon=1, goal=pose)
                        if plan_success:
                            full_path = np.vstack((full_path, partial_path))
                            self.waypoint_inds = np.concatenate((self.waypoint_inds, [full_path.shape[0] - 1]))
                    
                    if len(full_path.shape) > 1:
                        path_length = 0
                        curr_pose = full_path[0]
                        for i, next_pose in enumerate(full_path[1:]):
                            path_length += np.linalg.norm(next_pose[:2] - curr_pose[:2])
                            if path_length > self.max_path_length:
                                full_path = full_path[:i+1]
                                self.waypoint_inds = self.waypoint_inds[self.waypoint_inds < full_path.shape[0]]
                                break
                            else:
                                curr_pose = next_pose
                    
                    self.curr_path = full_path
                    self.goal = full_path[-1]
                    
                    path_msg = Path()
                    path_msg.header.frame_id = self.world_frame_id
                    
                    if len(full_path.shape) == 1:
                        self.add_waypoint(full_path, path_msg)
                    else:
                        for pose in full_path:
                            self.add_waypoint(pose, path_msg)
                    print("Revised path generated!")
                    
                    self.path_pub.publish(path_msg)
                    self.is_planning = False
        
    def get_neighborhood(self, pose):
        diffs_2d = self.vectorized_xy - pose[:2]
        dists_2d = np.linalg.norm(diffs_2d, axis=1)
        nearest_idx = np.argmin(dists_2d)
        cell_indx = np.nonzero(np.logical_and(self.SE2_radius - 2 * self.planning_map.resolution < dists_2d,
                                              dists_2d < self.SE2_radius))[0]
        return cell_indx, nearest_idx
        
    def check_reachability(self, cell_indx, nearest_indx):
        pose_rc = self.vectorized_rc[nearest_indx]
        reachable_rc = pose_rc[None, :]
        
        for cell_rc in self.vectorized_rc[cell_indx]:
            line = bresenham2d_with_intensities(pose_rc, cell_rc, self.distance_map.T)
            obstacle_indx = np.nonzero(line[:, 2] <= 0.1)[0]
            if obstacle_indx.shape[0] == 0:
                reachable_rc = np.vstack((reachable_rc, line[:, :2]))
            else:
                reachable_rc = np.vstack((reachable_rc, line[:obstacle_indx[0], :2]))

        reachable_rc = np.unique(reachable_rc, axis=0)
        
        return reachable_rc
        
    def compute_score(self, reachable_rc):
        scores = np.zeros((reachable_rc.shape[0], self.angular_samples))
        
        map_shape = self.probability_map.get_shape()
        
        for i, cell_rc in enumerate(reachable_rc):
            cell_rc = cell_rc.astype(int)
            dict_key_rc = tuple(cell_rc)
            if dict_key_rc in self.eval_cells:
                scores[i] = self.eval_cells[dict_key_rc]
            else:
                local_raytrace = self.raytrace_precomputed + cell_rc
                
                good_ind_r = np.logical_and(0 <= local_raytrace[:, :, 0], local_raytrace[:, :, 0] < map_shape[0])
                good_ind_c = np.logical_and(0 <= local_raytrace[:, :, 1], local_raytrace[:, :, 1] < map_shape[1])
                good_ind = np.logical_and(good_ind_r, good_ind_c)

                local_raytrace[np.nonzero(1 - good_ind)] = cell_rc
                local_raytrace_mask = np.logical_and(self.raytrace_mask, good_ind)
                local_raytrace_mask = np.hstack((local_raytrace_mask,
                                                 np.zeros((local_raytrace_mask.shape[0], 1), dtype=bool)))
                local_raytrace_mask_diff = - np.diff(local_raytrace_mask, axis=1, prepend=1)

                p_1 = self.probability_map.data[local_raytrace[:, :, 0], local_raytrace[:, :, 1]]
                p_0 = 1 - p_1

                p_1 = np.hstack((p_1, np.zeros((p_1.shape[0], 1))))
                p_1[(1 - local_raytrace_mask).astype(bool)] = 0
                p_1[local_raytrace_mask_diff.astype(bool)] = 1

                p_0 = np.hstack((np.ones((p_0.shape[0], 1)), p_0))

                p = np.cumprod(p_0, axis=1) * p_1

                cell_log_odd = self.log_odds_map_data[local_raytrace[:, :, 0], local_raytrace[:, :, 1]] -\
                               self.min_log_odd

                f_0 = self.f_precomputed[0, cell_log_odd]
                f_1 = self.f_precomputed[1, cell_log_odd]

                f_0 = np.hstack((np.zeros((f_0.shape[0], 1)), f_0))

                f_1 = np.hstack((f_1, np.zeros((f_1.shape[0], 1))))
                f_1[(1 - local_raytrace_mask).astype(bool)] = 0

                C = np.cumsum(f_0, axis=1) + f_1

                mut_info = np.sum(p * C, axis=1)

                scores[i] = mut_info + self.distance_coeff * np.log(1e-6 + self.distance_map[cell_rc[0], cell_rc[1]])
                self.eval_cells[dict_key_rc] = scores[i]
        
        return scores
        
    def compute_gradient(self, pose, cell_scores, reachable_rc):
        reachable_xy = rc_to_xy(reachable_rc, self.probability_map)
        diff_xy = reachable_xy - pose[:2]
        diff_xy_rep = np.repeat(diff_xy[:, :, None], self.precomputed_orientation_angles.shape[0], axis=2)

        diff_theta = wrap_angles(self.precomputed_orientation_angles - pose[2])
        diff_theta_rep = np.repeat(diff_theta[None, None, :], reachable_xy.shape[0], axis=0)

        diff_tensor = np.concatenate((diff_xy_rep, self.theta_coeff * diff_theta_rep), axis=1)

        delta = np.sqrt(diff_tensor[:, 0, :]**2 + diff_tensor[:, 1, :]**2 +
                        diff_tensor[:, 2, :]**2 / self.theta_coeff)[:, None, :]

        in_range_mask = (delta <= self.SE2_radius)

        cell_pose_scores = np.sum(cell_scores[:, self.precomputed_ray_inds], axis=2)[:, None, :]

        eta = np.sum(in_range_mask * (1 + np.cos(delta * np.pi / self.SE2_radius))) + 1e-6
        beta = np.sum(in_range_mask * cell_pose_scores * (1 + np.cos(delta * np.pi / self.SE2_radius))) / eta
        gradient = np.sum(in_range_mask * (cell_pose_scores - beta) *
                          np.sin(delta * np.pi / self.SE2_radius) / (delta + 1e-10) * diff_tensor,
                          axis=(0, 2)) / eta

        return gradient
        
    def find_frontiers(self, state):
        frontiers = extract_frontiers(occupancy_map=self.planning_map, kernel=self.kernel_cleanup)

        frontier_sizes = np.array([frontier.shape[0] if len(frontier.shape) > 1 else 1 for frontier in frontiers])
        valid_frontier_inds = np.argwhere(frontier_sizes >= self.min_frontier_size)

        frontier_goals = []
        frontier_size = []

        if valid_frontier_inds.shape[0] == 0:
            return None, None

        for v in valid_frontier_inds[:, 0]:
            f = frontiers[v]
            frontier_mean = np.mean(f, axis=0)

            frontier_position = f[np.argmin(np.linalg.norm(f - frontier_mean, axis=1))]
            frontier_orientation = wrap_angles(np.arctan2(frontier_position[1] - state[1], frontier_position[0] - state[0]))
            frontier_pose = np.array([frontier_position[0], frontier_position[1], frontier_orientation])

            frontier_goals.append(frontier_pose)
            frontier_size.append(f.shape[0] if len(f.shape) > 1 else 1)

        return frontier_goals, frontier_size
        
    def get_failsafe_path(self, state):
        # rospy.loginfo("111111111ENTERED FAILSAFE PATH")
        path = None
        self.plan_map_pub.publish(self.plan_map_msg)
        while path is None:
            random_start, start_sample_success = self.sample_free_pose()
            random_goal, goal_sample_success = self.sample_free_pose()
            if start_sample_success and goal_sample_success:
                plan_success, random_path = oriented_astar(start=random_start,
                                                           occupancy_map=self.planning_map,
                                                           footprint=self.footprint,
                                                           obstacle_values=[Costmap.OCCUPIED],
                                                           epsilon=1,
                                                           goal=random_goal)
            
                if plan_success and random_path.shape[0] > 1:
                    path_length = np.sum(np.linalg.norm(random_path[1:, :2] - random_path[:-1, :2], axis=1))
                    if path_length > self.goal_check_radius:
                        pixel_path = xy_to_rc(random_path, self.planning_map)
                        try:
                            # also, why check again here?
                                collision_inds = np.nonzero(self.planning_map.data[pixel_path[:, 0].astype(int),
                                                                                        pixel_path[:, 1].astype(int)] == Costmap.OCCUPIED)[0]
                                if collision_inds.shape[0] == 0:
                                        path = random_path
                        except IndexError:
                                continue
            rospy.sleep(0.2)
        assert path is not None, "Fail-safe path generation failed!"
        print(self.robot_name+" Fail-safe path generated!") 
        # data1 = self.planning_map.data.copy()
        # data2 = path.copy()
        return path
    
    def sample_free_pose(self):
        successful = False
        random_pose = None

        free_inds = np.nonzero(self.planning_map.data != Costmap.OCCUPIED)
        if free_inds[0].shape[0] == 0:
            return random_pose, successful

        while not successful:
            random_ind = np.random.randint(0, free_inds[0].shape[0])
            random_rc = np.array([free_inds[0][random_ind], free_inds[1][random_ind]])
            random_xy = rc_to_xy(random_rc, self.planning_map)
            random_orientation = np.random.uniform(0, 2*np.pi)
            random_pose = np.array([random_xy[0], random_xy[1], random_orientation])
            if not self.footprint.check_for_collision(random_pose, self.planning_map, unexplored_is_occupied=False):
                successful = True
            rospy.sleep(0.2)

        return random_pose, successful
    
    def add_waypoint(self, pose, path_msg):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        waypoint = PoseStamped()
        waypoint.header.frame_id = self.world_frame_id
        waypoint.header.stamp = rospy.Time.now()
        waypoint.pose.position.x = pose[0]
        waypoint.pose.position.y = pose[1]
        waypoint.pose.position.z = 0
        waypoint.pose.orientation.x = quaternion[0]
        waypoint.pose.orientation.y = quaternion[1]
        waypoint.pose.orientation.z = quaternion[2]
        waypoint.pose.orientation.w = quaternion[3]
        path_msg.poses.append(waypoint)
    
    def publish_path(self):
        path = self.team_path[1 + 3 * self.horizon * (self.agent_id - 1):1 + 3 * self.horizon * self.agent_id].reshape(self.horizon, 3)
        robot_pose = self.get_pose_from_tf(self.robot_frame_id)
        full_path = robot_pose[None, :]
        self.waypoint_inds = np.array([0], dtype=int)
        tmp_occ_map = self.planning_map.data.copy()
        plan_map_msg = OccupancyGrid()
        plan_map_msg.header = Header()
        plan_map_msg.header.stamp = rospy.Time.now()
        plan_map_msg.header.frame_id = self.world_frame_id
        plan_map_msg.info.resolution = self.planning_map.resolution
        plan_map_msg.info.width = self.planning_map.data.shape[1]
        plan_map_msg.info.height = self.planning_map.data.shape[0]
        plan_map_msg.info.origin.position.x = self.planning_map.origin[0]
        plan_map_msg.info.origin.position.y = self.planning_map.origin[1]
        tmp_occ_map_int = np.zeros_like(tmp_occ_map, dtype=np.int8) 
        tmp_occ_map_int[tmp_occ_map == Costmap.UNEXPLORED] = -1
        tmp_occ_map_int[tmp_occ_map == Costmap.FREE] = 0    
        tmp_occ_map_int[tmp_occ_map == Costmap.OCCUPIED] = 100
        tmp_occ_map_int = np.flipud(tmp_occ_map_int)
        plan_map_msg.data = tmp_occ_map_int.flatten().tolist()
        self.plan_map_msg = plan_map_msg
        self.plan_map_pub1.publish(plan_map_msg)
        map_visulize(self.planning_map.data, self.planning_map.resolution, self.planning_map.origin)
        plt.savefig('/root/ws/map/map%d.png'%self.map_counter)
        plt.clf()
        self.map_counter += 1
        # plt.show()
        for pose in path:
            print('waypoint: ', pose)
            plan_success, partial_path = oriented_astar(start=full_path[-1],
                                                        occupancy_map=self.planning_map,
                                                        footprint=self.footprint,
                                                        obstacle_values=[Costmap.OCCUPIED],
                                                        epsilon=1,
                                                        goal=pose)
            if plan_success:
                # Why we check the collision here when plan_success is True? Plan scale in cpp function is default to 1
                pixel_path = xy_to_rc(partial_path, self.planning_map)
                try:
                    collision_inds = np.nonzero(self.planning_map.data[pixel_path[:, 0].astype(int),
                                                pixel_path[:, 1].astype(int)] == Costmap.OCCUPIED)[0]
                    if collision_inds.shape[0] == 0:
                            full_path = np.vstack((full_path, partial_path))
                            self.waypoint_inds = np.concatenate((self.waypoint_inds, [full_path.shape[0] - 1]))
                    else:
                        raise ValueError('Collision detected in the path!')
                except IndexError:
                    print('IndexError11111111111111111111111111111111111')
                    continue
        # assert full_path.shape[0] > 1, "Path generation failed!"            
        if len(full_path.shape) > 1:
            path_length = 0
            curr_pose = full_path[0]
            for i, next_pose in enumerate(full_path[1:]):
                path_length += np.linalg.norm(next_pose[:2] - curr_pose[:2])
                if path_length > self.max_path_length:
                    full_path = full_path[:i+1]
                    self.waypoint_inds = self.waypoint_inds[self.waypoint_inds < full_path.shape[0]]
                    break
                else:
                    curr_pose = next_pose
        
        self.curr_path = full_path
        self.goal = full_path[-1]
        
        path_msg = Path()
        path_msg.header.frame_id = self.world_frame_id
        
        if len(full_path.shape) == 1:
            self.add_waypoint(full_path, path_msg)
            rospy.loginfo("{}: warning, path with single pose!".format(self.robot_name))
        else:
            for pose in full_path:
                self.add_waypoint(pose, path_msg)
        
        self.path_pub.publish(path_msg)
        
        rospy.loginfo("{}: Brand New path published!".format(self.robot_name))


def main(args):
    rospy.init_node('ga_multi_exploration', anonymous=True)
    ga_multi_exp_agent = GradAscentMultiExplorationAgent()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Exploration stopped!")


if __name__ == '__main__':
    main(sys.argv)
