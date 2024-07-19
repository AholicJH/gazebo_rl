#!/usr/bin/env python

import gym
from gym import register
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3 import SAC

from frobs_rl.common import ros_gazebo
import rospy


register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=1000,
)


if __name__ == '__main__':
    



    rospy.logwarn("Start")
    rospy.init_node('train_Pavs03_predict')
    # 定义你的环境，这里假设是一个 OpenAI Gym 的环境
    env = gym.make('PavsReachAvoidEnv-v0')  # 替换为你的环境名称或者自定义环境
    
    # 加载训练好的模型
    # 加载模型
    model_path = '/home/jianghan/gazebo_rl/gazebo_rl/src/pavs_rl/models/sac/trained_model_17_07_2024_14_27_02.zip'
    model = SAC.load(model_path)

    # 预测使用模型
    obs = env.reset()  # 重置环境
    done = False
    while not done:
        action, _states = model.predict(obs, deterministic=True)  # 预测动作
        obs, reward, done, info = env.step(action)  # 执行动作并观察环境反馈
        print('state = {0}; reward = {1}; done = {2}; info = {3}'.format(obs, reward, done, info))
    
    # 关闭环境
    ros_gazebo.gazebo_pause_physics()
    env.close()
    # 停止仿真环境
    # ros_gazebo.gazebo_pause_physics()
