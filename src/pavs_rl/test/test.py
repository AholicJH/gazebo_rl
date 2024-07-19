import rospy
from nav_msgs.msg import Odometry
import numpy as np

class YourClass:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('position_logger', anonymous=True)

        # 初始化位置变量
        self.pos_prev = np.zeros(3)  # 初始时设置为零向量
        self.pos_curr = np.zeros(3)  # 初始时设置为零向量

        # 订阅机器人位置信息的话题
        rospy.Subscriber('/pav_s03/odom', Odometry, self.position_callback)

    def position_callback(self, msg):
        # 获取当前位置信息
        self.current_position = np.array([msg.pose.pose.position.x,
                                          msg.pose.pose.position.y,
                                          msg.pose.pose.position.z])

        # 更新前一个时刻的位置
        self.pos_prev = self.pos_curr.copy()

        # 更新当前时刻的位置
        self.pos_curr = self.current_position.copy()

        # 打印日志，验证位置信息
        rospy.loginfo(f"Previous position: {self.pos_prev}")
        rospy.loginfo(f"Current position: {self.pos_curr}")

if __name__ == '__main__':
    try:
        your_instance = YourClass()
        rospy.spin()  # 保持节点运行，等待订阅的消息到来
    except rospy.ROSInterruptException:
        pass
