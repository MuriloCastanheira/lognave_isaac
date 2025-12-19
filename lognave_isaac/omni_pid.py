#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoToGoalOmni(Node):
    def __init__(self):
        super().__init__('go_to_goal_omni')

        # Publisher e Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Timer
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Estado do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Waypoints
        self.x_goal = [7.0, 7.0, 0.0, 0.0]
        self.y_goal = [0.0, 7.0, 7.0, 0.0]
        self.goal_state = 0

        # Ganhos
        self.kp_xy = 1.0
        self.kp_ang = 2.0

        # Limites
        self.max_v = 0.5
        self.max_w = 0.5

        self.goal_tolerance = 0.1

        self.get_logger().info('🚀 Go-to-goal omnidirecional iniciado')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        dx = self.x_goal[self.goal_state] - self.x
        dy = self.y_goal[self.goal_state] - self.y

        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.goal_tolerance:
            if self.goal_state < len(self.x_goal) - 1:
                self.goal_state += 1
                self.get_logger().info(f'➡️ Indo para ponto {self.goal_state}')
            else:
                self.stop_robot()
                self.get_logger().info('🎯 Todos os pontos alcançados')
                return

        # Erro no frame do robô
        vx =  math.cos(self.theta) * dx + math.sin(self.theta) * dy
        vy = -math.sin(self.theta) * dx + math.cos(self.theta) * dy

        # Controle proporcional
        vx = self.kp_xy * vx
        vy = self.kp_xy * vy

        # Saturação
        vx = max(min(vx, self.max_v), -self.max_v)
        vy = max(min(vy, self.max_v), -self.max_v)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = -vy
        cmd.angular.z = 0.0  # pode adicionar controle de orientação se quiser

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalOmni()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
