import numpy as np
import rospkg
import rospy
from stable_baselines3 import TD3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class RLAgent:
    def __init__(self):
    

    
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("pavs_rl")
        save_path = pkg_path + "/models/TD3/"+"trained_model_06_08_2024_11_44_43"
        self.model = TD3.load(save_path)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_position = None
        # 设置初始目标点
        self.target_position = np.array([2.000000, -2.000000, 0.000131], dtype=np.float32)  # 目标位置

    def laser_scan_callback(self, data):
        # 处理激光雷达数据
        self.laser_data = self.process_scan(data)
        # 如果已接收到里程计数据，生成观测值并进行预测
        if self.current_position is not None and self.target_position is not None:
            observation = self.create_observation()
            action, _ = self.model.predict(observation)
            self.send_action(action)

    def odom_callback(self, msg):
        # 处理里程计数据
        self.current_position = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])

    def process_scan(self, msg):
        # 实现处理激光雷达数据的逻辑
        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = msg.range_max  # 将无穷远值替换为最大测量距离
        # ranges = np.append(ranges, 0)  # 在数组末尾添加一个零
        return ranges
        # return np.zeros(60)

    def create_observation(self):
        # 将激光雷达数据和里程计数据结合形成观测值
        observation = np.concatenate((self.current_position, self.target_position, np.ravel(self.laser_data)))
        return observation

    def send_action(self, action):
        # 将动作转换为控制指令并发布
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        self.pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('rl_agent_node')
    agent = RLAgent()
    rospy.spin()
