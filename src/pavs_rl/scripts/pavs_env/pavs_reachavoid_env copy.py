#!/usr/bin/env python

import gym
import math
from gym.envs.registration import register
import rospkg
from frobs_rl.common import ros_gazebo
from frobs_rl.common import ros_controllers
from frobs_rl.common import ros_node
from frobs_rl.common import ros_spawn
from frobs_rl.envs.robot_BasicEnv import RobotBasicEnv
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from gym import spaces




class PavsReachAvoidEnv(RobotBasicEnv):

    def __init__(self):

        rospy.loginfo(f"\033[94m PAVS RL Environment initializing.\033[0m")

        launch_gazebo = True
        gazebo_init_paused = True
        gazebo_use_gui = True
        gazebo_recording = False
        gazebo_freq = 100
        '''
        如果使用自定义世界启动 Gazebo，请设置相应的环境变量。
        '''
        
        pkg_path = '/home/jianghan/gazebo_rl/gazebo_rl/src/car-like-robot-gazebo/robot_gazebo'
        world_path = pkg_path +'/worlds/ROS-Academy.world'
        world_pkg = 'robot_gazebo'
        world_filename = 'ROS-Academy.world'
        
        gazebo_max_freq = None
        gazebo_timestep = None
        spawn_robot = True
        model_name_in_gazebo = "pav_s00"
        namespace = "/pav_s00"
        pkg_name = 'robot_gazebo'
        urdf_file = 'pav_s00.xacro'
        urdf_folder ='/urdf/pav_s00'
        controller_file = 'controller_manager/pav_s00_motor_ctrl_config.yaml'
        controller_list = ['left_front_wheel_velocity_controller',
                           'right_front_wheel_velocity_controller',
                           'right_rear_wheel_velocity_controller',
                           'left_rear_wheel_velocity_controller',
                           'left_steering_hinge_position_controller',
                           'right_steering_hinge_position_controller',
                           'joint_state_controller']
        urdf_xacro_args = None
        rob_state_publisher_max_freq = None
        rob_st_term = False
        model_pos_x = 0.0
        model_pos_y = 0.0
        model_pos_z = 0.0
        model_ori_x = 0.0
        model_ori_y = 0.0
        model_ori_z = 0.0
        model_ori_w = 0.0
        reset_controllers = False
        reset_mode = 1
        step_mode = 1
        num_gazebo_steps = 1

        super(PavsReachAvoidEnv, self).__init__(launch_gazebo=launch_gazebo, gazebo_init_paused=gazebo_init_paused, 
                                                gazebo_use_gui=gazebo_use_gui, gazebo_recording=gazebo_recording,
                                                gazebo_freq=gazebo_freq, world_path=world_path, world_pkg=world_pkg,
                                                world_filename=world_filename, gazebo_max_freq=gazebo_max_freq,
                                                gazebo_timestep=gazebo_timestep, spawn_robot=spawn_robot, 
                                                model_name_in_gazebo=model_name_in_gazebo,namespace=namespace, pkg_name=pkg_name,
                                                urdf_file=urdf_file, urdf_folder=urdf_folder,  controller_file=controller_file, 
                                                controller_list=controller_list, urdf_xacro_args=urdf_xacro_args,
                                                model_pos_x=model_pos_x, model_pos_y=model_pos_y, model_pos_z=model_pos_z,
                                                model_ori_x=model_ori_x, model_ori_y=model_ori_y, model_ori_z=model_ori_z, model_ori_w=model_ori_w,
                                                reset_controllers=reset_controllers, reset_mode=reset_mode, step_mode=step_mode,
                                                num_gazebo_steps=num_gazebo_steps)

        self.pos_curr = [model_pos_x, model_pos_y, model_pos_z]

        self.ori_position =[model_ori_x, model_ori_y, model_ori_z]

         # 动作空间定义 (线速度和角速度)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),  # 线速度最小值, 角速度最小值
            high=np.array([1.0, 1.0]),   # 线速度最大值, 角速度最大值
            dtype=np.float32
        )
        
        # 观测空间定义（例如，位置、速度、激光雷达数据）
        num_lidar_readings = 360  # 假设激光雷达有360个角度的读数
        self.observation_space = spaces.Box(
            low=np.concatenate(([-np.inf]*3, [-np.inf]*3, [-1.0]*num_lidar_readings)),
            high=np.concatenate(([np.inf]*3, [np.inf]*3, [1.0]*num_lidar_readings)),
            dtype=np.float32
        )
        rospy.loginfo("Finished Init of PAVS RL Environment")

  
    def _get_observation(self):
        """
        Function to get the observation from the environment.
        The observation includes: current position, goal position, laser scan.

        param:
        ------
            - None

        return:
        -------
            - observation: list containing current position, goal position, and laser scan
        """
         
        # 设置订阅者获取当前位置（从里程计消息）
        self.current_position = None
        rospy.Subscriber('/odom', Odometry, odom_callback)
        

         # 设置订阅者获取激光扫描数据
        self.laser_scan = None
        rospy.Subscriber('/scan', LaserScan, laser_scan_callback)


        def odom_callback(msg):
            position = msg.pose.pose.position
            self.current_position = [position.x, position.y, position.z]

        def laser_scan_callback(msg):
            self.laser_scan = np.array(msg.ranges)
        
         # 等待直到获取到当前位置和激光扫描数据
        while self.pos_curr is None or self.laser_scan is None:
            rospy.sleep(0.1)

        # 组合观测值
        observation = {
            'current_position': self.current_position,
            'goal_position': self.ori_position,
            'laser_scan': self.laser_scan
        }

        # 返回观测值
        return observation

        # self.pos_goal = self.set_goal([1.0, 1.0, 0.0])
        # self.pos_goal = [1.0, 1.0, 0.0]
        # self.scan = []

        # # TODO, /scan topic, downsample: a, configuration; b, array downsample

        # def odom_callback(msg):
        #     position = msg.pose.pose.position
        #     self.pos_curr = [position.x, position.y, position.z]

        # def scan_callback(msg):
        #     ranges = np.array(msg.ranges)
        #     self.scan = ranges[::10]

        # rospy.Subscriber('/odom', Odometry, odom_callback)
        # rospy.Subscriber('/scan', LaserScan, scan_callback)

        # while not self.pos_curr or not self.scan:
        #     rospy.sleep(0.1)

        # self.observation = [self.pos_curr, self.pos_goal, self.scan.tolist()]

        # return self.observation

    def _get_reward(self):
        """
        Function to get the reward from the environment.

        params:
        ------
            - None

        returns:
        --------
            - is_collision(bool): True means collision, False means no collision.
        """
        self.reward = 0.0
       
        dist_curr = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal[:2]))
        dist_next = np.linalg.norm(np.array(self.pos_next[:2]) - np.array(self.pos_goal[:2]))

        self.reward_goal = dist_curr - dist_next  # goal reward. close to goal, reward_goal is larger, else negative.
        self.reward += self.reward_goal

        if self._obstacle_detection():
            self.reward_obs = -1  # obstacle reward, negative value.
            self.reward += self.reward_obs
            is_collision = True
        else:
            is_collision = False

        return is_collision

    def _obstacle_detection(self):
        """
        Function to detect whether collision in the environment.

        params:
        ------
            - None

        returns:
        --------
            - is_collision(bool): True means collision, False means no collision.
        """
        is_collision = False

        d_min = float('inf')
        for pos in self.pos_obs:
            d = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(pos[:2]))
            if d < d_min:
                d_min = d

        if d_min < 0.1:
            is_collision = True

        return is_collision

    def _check_if_done(self):
        """
        Function to check if the episode is done.

        If the episode has a success condition then set done as:
            self.info['is_success'] = 1.0

        params:
        ------
            - goal_threshold： The threshold of goal, and now let this hyper-parameter is 0.1.
            - obstacle_threshold: The threshold of obstacle, and now let this hyper-parameter is 0.1.
        returns:
        --------
            - if_done(bool): True means done, False means not done.

        """
        done = False
        self.info = {'is_success': 0.0}

        goal_threshold = 0.1
        distance_goal = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal))

        if distance_goal < goal_threshold:
            done = True
            self.info['is_success'] = 1.0
            return done

        obstacle_threshold = 0.2

        for obs in self.pos_obs:
            distance_obstacle = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(obs))
            if distance_obstacle < obstacle_threshold:
                done = True
                return done

        raise done

    def _set_obstacle(self, positions):
        """
        Function to set obstacles position.
        """
        self.pos_obs = []
        for pos in positions:
            self.pos_obs.append(pos)

    def _set_goal(self, pos):
        """
        Function to set goal position.
        """
        self.pos_goal = pos

    def _reset_gazebo(self):
        """
        Function to reset the gazebo simulation.
        """

        # If resetting world (Does not reset time)
        if self.reset_mode == 1:
            ros_gazebo.gazebo_reset_world()

        # If resetting simulation (Resets time)
        elif self.reset_mode == 2:
            ros_gazebo.gazebo_reset_sim()

        if self.reset_controllers:
            ros_gazebo.gazebo_unpause_physics()
            ros_controllers.reset_controllers_srv(self.controllers_list, ns=self.namespace)
            ros_gazebo.gazebo_pause_physics()

        ros_gazebo.gazebo_unpause_physics()
        # self._check_subs_and_pubs_connection()
        # self._set_episode_init_params()
        ros_gazebo.gazebo_pause_physics()

