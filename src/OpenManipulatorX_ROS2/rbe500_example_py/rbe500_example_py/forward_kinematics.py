"""
cmds
cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbe500_example_py
source install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO749-if00-port0

cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rbe500_example_py forward_kinematics

cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once /calculate_forward_pose std_msgs/msg/Float32MultiArray "{data: [0.5, 0.0, 0.0, 0.0]}"
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition

from std_msgs.msg import Float32MultiArray

# length constants (mm)
L1 = 96.326
L2 = 128 
L3 = 124 # check if this is correct with the actual robot
L4 = 133.4

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

        # create subscription for calculating forward kinematics
        self.fk_subscription = self.create_subscription(
            Float32MultiArray,
            'calculate_forward_pose',
            self.calculate_forward_pose_callback,
            10)

        # create client for setting joint position
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

    def calculate_forward_pose_callback(self, msg):
        # message angles
        q1, q2, q3, q4 = msg.data

        # frame transformation (FIX ONCE HAVE TABLE)
        a1 = forward_kinematics(0, q1, L1, math.radians(270))
        a2 = forward_kinematics(L2, q2, 0, 0)
        a3 = forward_kinematics(L3, q3, 0, 0)
        a4 = forward_kinematics(L4, q4, 0, 0)

        t04 = np.array(a1) @ np.array(a2) @ np.array(a3) @ np.array(a4)

        # print matrix
        self.get_logger().info(f'T04 =\n{t04}')

        # send request to set joint position
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = [q1, q2, q3, q4, 0.0]
        request.path_time = 5.0
        
        self.future = self.client.call_async(request)

# main
def main(args=None):
    rclpy.init(args=args)
    node = BasicRobotControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

