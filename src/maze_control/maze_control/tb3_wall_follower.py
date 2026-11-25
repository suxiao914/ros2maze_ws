import csv
import os
from math import pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

CSV_PATH = os.path.expanduser('~/tb3_trajectory.csv')      # ①-3 轨迹数据存这里


class LeftWallFollower(Node):
    """左墙跟随：保持左侧墙面距离；前方过小原地右转"""

    def __init__(self):
        super().__init__('tb3_wall_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ===== ① 新增：里程计订阅 + Path 发布 + CSV 记录 =====
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'     # RViz 里 Fixed Frame 要设 odom
        self.last_path_t = 0.0
        self.path_started = False
        self.path_points = 0

        self.csv_file = open(CSV_PATH, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t', 'x', 'y'])
        # ==================================================

        self.target_left = 0.4      # 期望左侧墙距 m
        self.front_danger = 0.35    # 前方危险距离 m
        self.v = 0.15               # 直行速度
        self.w = 0.6                # 转弯角速度

        self.front = 10.0
        self.left = 10.0
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        n = len(msg.ranges)
        inc = msg.angle_increment
        a_min = msg.angle_min

        def dist_at(angle_deg):
            # 分区域读取：角度换算成 ranges 索引
            i = int((angle_deg * pi / 180.0 - a_min) / inc)
            i = max(0, min(n - 1, i))
            return msg.ranges[i]

        # 前方 0°±15°；左侧取 60°~90°（正左侧附近）
        self.front = min(dist_at(0), dist_at(15), dist_at(-15))
        self.left = min(dist_at(60), dist_at(75), dist_at(90))

    # ===== ② 新增：里程计回调，负责画轨迹 =====
    def odom_callback(self, msg):
        # 用 odom 自带时间戳，避免 use_sim_time / /clock 未就绪时时间恒为 0
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.path_started and (t - self.last_path_t) < 0.1:
            return                      # 10 Hz 采样，够了
        self.path_started = True
        self.last_path_t = t

        ps = PoseStamped()
        ps.header = msg.header          # frame_id = odom
        ps.pose = msg.pose.pose
        self.path_msg.poses.append(ps)
        self.path_msg.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path_msg)     # 整条重发，否则 RViz 只剩一个点

        p = msg.pose.pose.position
        self.csv.writerow(['%.3f' % t, '%.4f' % p.x, '%.4f' % p.y])
        self.path_points += 1
        if self.path_points % 50 == 0:
            self.csv_file.flush()
            self.get_logger().info('轨迹点数: %d' % self.path_points)
    # ==========================================

    def control_loop(self):
        twist = Twist()
        if self.front < self.front_danger:
            # 前方距离过小 → 原地右转
            twist.linear.x = 0.0
            twist.angular.z = -self.w
        elif self.left > self.target_left + 0.1:
            # 左侧距离偏大 → 向左微调
            twist.linear.x = self.v
            twist.angular.z = 0.4
        elif self.left < self.target_left - 0.1:
            # 左侧太近 → 向右微调（防撞墙）
            twist.linear.x = self.v
            twist.angular.z = -0.3
        else:
            # 左侧保持设定距离 → 直线前进
            twist.linear.x = self.v
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # ===== ③ 新增：退出时收尾，保证 CSV 完整 =====
    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info('轨迹数据已保存: %s（共 %d 点）' % (CSV_PATH, self.path_points))


def main():
    rclpy.init()
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
