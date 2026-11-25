#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from math import sqrt, atan2, pi, sin, cos, degrees

# 轨迹保存位置
CSV_PATH = os.path.expanduser('~/tb3_trajectory.csv')
PNG_PATH = os.path.expanduser('~/tb3_trajectory.png')


class WaypointController(Node):
    def __init__(self):
        super().__init__('tb3_waypoints')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.Kp_dist = 0.3
        self.Kp_ang = 0.8
        self.max_v = 0.2
        self.max_w = 1.0
        self.threshold = 0.05   # 航点到达切换阈值

        # 矩形航点列表，可自行修改
        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.wp_index = 0

        # 新增 ：RViz 轨迹话题 
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        

        # 新增 ：记录 + 保存 
        self.trail = []                                    
        self.csv_file = open(CSV_PATH, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t', 'x', 'y', 'yaw_deg', 'wp_index'])

        self.t0 = self.get_clock().now().nanoseconds / 1e9  # 起始时刻
        self.last_log = -1.0
        self.log_period = 0.1                               # 10 Hz 记一个点
        self.plot_saved = False
        # ===========================================

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)

        #新增 ：采样 → 发 Path + 存 CSV
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        if t - self.last_log >= self.log_period:
            self.last_log = t
            self.trail.append((self.x, self.y))

            self.csv.writerow([f'{t:.3f}', f'{self.x:.4f}', f'{self.y:.4f}',
                               f'{degrees(self.theta):.1f}', self.wp_index])
            self.csv_file.flush()

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = msg.header.stamp
            pose.pose = msg.pose.pose          # 赋值是深拷贝，安全

            self.path_msg.header.stamp = msg.header.stamp
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)   # 每次发完整路径
        

    def control_loop(self):
        if self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            if not self.plot_saved:                       
                self.get_logger().info("全部航点完成，正在保存轨迹...")
                self.save_plot()
            return

        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        if dist_err > self.threshold:
            twist.linear.x = self.Kp_dist * dist_err
            twist.angular.z = self.Kp_ang * theta_err
            twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
            twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        else:
            self.get_logger().info(f"到达航点{self.wp_index}:({gx},{gy})，切换下一个航点")
            self.wp_index += 1
        self.cmd_pub.publish(twist)

    # 新增 ：画图并保存
    def save_plot(self):
        if self.plot_saved or len(self.trail) < 2:
            return
        self.plot_saved = True

        try:
            import matplotlib
            matplotlib.use('Agg')          # 不弹窗，直接写文件
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn(
                '没装 matplotlib，跳过画图。执行: sudo apt install python3-matplotlib')
            return

        xs = [p[0] for p in self.trail]
        ys = [p[1] for p in self.trail]

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(xs, ys, '-', color='tab:blue', lw=2, label='actual path')
        ax.plot(xs[0], ys[0], 'o', color='green', ms=9, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='red', ms=9, label='end')

        # 理想矩形（闭合）
        wx = [p[0] for p in self.waypoints] + [self.waypoints[0][0]]
        wy = [p[1] for p in self.waypoints] + [self.waypoints[0][1]]
        ax.plot(wx, wy, 'k--', lw=1, marker='x', ms=9, label='waypoints')

        ax.set_aspect('equal')             
        ax.grid(True, linestyle=':')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('TurtleBot3 rectangle trajectory')
        ax.legend(loc='best')

        fig.savefig(PNG_PATH, dpi=200, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'轨迹图已保存: {PNG_PATH}')
        self.get_logger().info(f'轨迹数据已保存: {CSV_PATH}')
   


def main():
    rclpy.init()
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()          
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
