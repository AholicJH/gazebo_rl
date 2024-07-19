#!/usr/bin/env python

from pickle import POP
import sys
import gym
from frobs_rl.models.ppo import PPO
import rospy
import rospkg
from frobs_rl.common import ros_gazebo, ros_node
from frobs_rl.models.sac import SAC
from frobs_rl.models.td3 import TD3
from frobs_rl.wrappers.PavsActionWrapper import PavsActionWrapper
from frobs_rl.wrappers.PavsObservWrapper import PavsObservWrapper
from frobs_rl.wrappers.PavsTimeLimitWrapper import PavsTimeLimitWrapper
from gym import register

register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=1000,
)


if __name__ == '__main__':

   
    # ros_node.ros_kill_all_processes()
  
    # ros_gazebo.launch_Gazebo(paused=False, gui=False)
  
    rospy.logwarn("Start")
    rospy.init_node('train_irb120_reacher')
    env = gym.make('PavsReachAvoidEnv-v0')
    env.reset()

    # 假设设置最大步数
    # 500次eposide训练一次
    # max_steps_per_episode = 500
    # num_steps = 0

    # # while num_steps < max_steps:
    # while True:
    #     # 渲染画面
    #     env.render()
    #     # 从动作空间随机获取一个动作
    #     action = env.action_space.sample()
    #     # rospy.loginfo("已经获取动作")
    #     # agent与环境进行一步交互
    #     state, reward, done, info = env.step(action)
    #     print('state = {0}; reward = {1}; done = {2}; info = {3}'.format(state, reward, done, info))

    #     num_steps += 1

    #      # 检查是否达到终止条件或最大步数
    #     if done or num_steps >= max_steps_per_episode:
    #         print("Episode finished after {} steps".format(num_steps))
    #         env.reset()
    #         num_steps = 0  # 重置步数计数器
            
           

    # 环境结束
    # env.close()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    rospy.loginfo("pkg_path:"+pkg_path)
    #模型将要保存的路径ff
    save_path = pkg_path + "/models/sac/" 
    #日志将要保存的路径
    log_path = pkg_path + "/logs/sac/"               # Path where the logs will be saved
    model = SAC(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="sac_config.yaml")
    # model = PPO(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="ppo_config.yaml")
    model.train()
    model.save_model()
    model.close_env()

    # ros_gazebo.gazebo_pause_physics()

    sys.exit()