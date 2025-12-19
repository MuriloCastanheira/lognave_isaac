#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class OmniKinematics(Node):

    def __init__(self):
        super().__init__('omni_kinematics_joint_state')

        # ===== PARÂMETROS DO ROBÔ =====
        self.r = 0.05      # raio da roda (m)
        self.L = 0.20      # metade do comprimento (m)
        self.W = 0.20      # metade da largura (m)

        self.k = self.L + self.W
        self.s = math.sqrt(2) / 2

        # nomes das juntas (na ordem certa!)
        self.joint_names = [
            'first_wheel_joint',
            'second_wheel_joint',
            'third_wheel_joint',
            'fourth_wheel_joint'
        ]

        # subscriber em cmd_vel
        self.create_subscription( Twist, '/cmd_vel', self.cmd_vel_callback, 1)

        # publisher de joint_states
        self.js_pub = self.create_publisher( JointState, '/joint_states', 1)

        self.get_logger().info('Omni kinematics node started')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z

        
        # ===== CINEMÁTICA INVERSA =====
        # k = self.L + self.W

        # w1 = (vx - vy - k * w) / self.r
        # w2 = (vx + vy + k * w) / self.r
        # w3 = (vx + vy - k * w) / self.r
        # w4 = (vx - vy + k * w) / self.r


        # w1 = ( self.s*vx + self.s*vy - self.k*w ) / self.r
        # w2 = ( self.s*vx - self.s*vy + self.k*w ) / self.r
        # w3 = ( self.s*vx - self.s*vy - self.k*w ) / self.r
        # w4 = ( self.s*vx + self.s*vy + self.k*w ) / self.r

        w1 = -( self.s*vx + self.s*vy - self.k*w ) / self.r   # frente esquerda  /
        w2 = ( self.s*vx - self.s*vy + self.k*w ) / self.r   # frente direita   \
        w3 = ( self.s*vx - self.s*vy - self.k*w ) / self.r   # trás esquerda    \
        w4 = -( self.s*vx + self.s*vy + self.k*w ) / self.r   # trás direita     /
        print(w1, w2, w3, w4)

        # ===== JOINT STATE =====
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        
        js.velocity = [w1, w2, w3, w4]

        # position e effort podem ficar vazios
        js.position = []
        js.effort = []

        self.js_pub.publish(js)


def main():
    rclpy.init()
    node = OmniKinematics()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
