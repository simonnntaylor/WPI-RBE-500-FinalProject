import math

import numpy as np

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys

from std_msgs.msg import Float32MultiArray

# calculate DH matrix
def forward_kinematics(a, q, d, aa):
    ct = math.cos(q)
    st = math.sin(q)
    ca = math.cos(aa)
    sa = math.sin(aa)

    # DH matrix
    return [[ct, -st*ca, st*sa, a*ct],
         [st, ct*ca, -ct*sa, a*st],
         [0, sa, ca, d],
         [0, 0, 0, 1]]

class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('calculate_pose')

        # create subscription for forward kinematics
        self.fk_subscription = self.create_subscription(
            Float32MultiArray,
            'calculate_forward_pose',
            self.calculate_forward_pose_callback,
            10)

    def calculate_forward_pose_callback(self, msg):
        # message angles
        q1, q2, q3 = msg.data

        # frame transformation
        a1 = forward_kinematics(0, q1, D1_CONSTANT, math.radians(270))
        a2 = forward_kinematics(D2_CONSTANT, q2, 0, 0)
        a3 = forward_kinematics(D3_CONSTANT, q3, 0, 0)

        t03 = np.array(a1) @ np.array(a2) @ np.array(a3)

        # print matrix
        self.get_logger().info(f'T03 =\n{t03}')

def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

# main
def main(args=None):
    rclpy.init(args=args)
    node = BasicRobotControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

