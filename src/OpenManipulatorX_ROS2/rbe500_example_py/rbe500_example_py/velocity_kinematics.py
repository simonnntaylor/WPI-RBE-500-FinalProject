"""
cmds
colcon build --packages-select rbe500_interfaces rbe500_example_py
source install/setup.bash
ros2 run rbe500_example_py velocity_kinematics

ros2 service call /joint_to_ee_velocity rbe500_interfaces/srv/JointToEEVelocity "{q: [0.0, 0.0, 0.0, 0.0], q_dot: [0.1, 0.0, 0.0, 0.0]}"
ros2 service call /ee_to_joint_velocity rbe500_interfaces/srv/EEToJointVelocity "{q: [0.0, -0.5, 0.5, 0.0], ee_vel: [0.0, 10.0, 0.0]}"
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rbe500_interfaces.srv import JointToEEVelocity, EEToJointVelocity

# length constants (mm)
L1 = 96.326
L2 = math.sqrt(128**2 + 24**2)  # link 2 has a 24 mm offset
L3 = 124
L4 = 133.4

# angle offset on joint 2 and 3 because of the 24 mm offset
OFFSET = math.atan2(128, 24)

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

# calculate the 6x4 jacobian
def calculate_jacobian(q1, q2, q3, q4):
    # frame transformation
    a1 = forward_kinematics(0, q1, L1, math.radians(270))
    a2 = forward_kinematics(L2, q2 - OFFSET, 0, 0)
    a3 = forward_kinematics(L3, q3 + OFFSET, 0, 0)
    a4 = forward_kinematics(L4, q4, 0, 0)

    t01 = np.array(a1)
    t02 = t01 @ np.array(a2)
    t03 = t02 @ np.array(a3)
    t04 = t03 @ np.array(a4)

    # z axis of each frame
    z0 = np.array([0, 0, 1])
    z1 = t01[0:3, 2]
    z2 = t02[0:3, 2]
    z3 = t03[0:3, 2]

    # origin of each frame
    o0 = np.array([0, 0, 0])
    o1 = t01[0:3, 3]
    o2 = t02[0:3, 3]
    o3 = t03[0:3, 3]
    o4 = t04[0:3, 3]

    # revolute joints: Jv = z x (o4 - o), Jw = z
    J = np.zeros((6, 4))
    J[0:3, 0] = np.cross(z0, o4 - o0)
    J[0:3, 1] = np.cross(z1, o4 - o1)
    J[0:3, 2] = np.cross(z2, o4 - o2)
    J[0:3, 3] = np.cross(z3, o4 - o3)
    J[3:6, 0] = z0
    J[3:6, 1] = z1
    J[3:6, 2] = z2
    J[3:6, 3] = z3

    return J

class VelocityKinematics(Node):
    def __init__(self):
        super().__init__('velocity_kinematics')

        # create service for joint velocities -> end effector velocity
        self.joint_to_ee_service = self.create_service(
            JointToEEVelocity,
            'joint_to_ee_velocity',
            self.joint_to_ee_callback)

        # create service for end effector velocity -> joint velocities
        self.ee_to_joint_service = self.create_service(
            EEToJointVelocity,
            'ee_to_joint_velocity',
            self.ee_to_joint_callback)

    def joint_to_ee_callback(self, request, response):
        # request angles and joint velocities
        q1, q2, q3, q4 = request.q
        q_dot = np.array(request.q_dot)

        J = calculate_jacobian(q1, q2, q3, q4)

        # ee_vel = J * q_dot
        ee_vel = J @ q_dot

        # print velocity
        self.get_logger().info(f'ee_vel =\n{ee_vel}')

        response.ee_vel = ee_vel.tolist()
        return response

    def ee_to_joint_callback(self, request, response):
        # request angles and end effector velocity
        q1, q2, q3, q4 = request.q
        ee_vel = np.array(request.ee_vel)

        J = calculate_jacobian(q1, q2, q3, q4)

        # only use the linear part since the robot only has 4 joints
        Jv = J[0:3, :]

        # q_dot = pinv(Jv) * ee_vel
        q_dot = np.linalg.pinv(Jv) @ ee_vel

        # print joint velocities
        self.get_logger().info(f'q_dot =\n{q_dot}')

        response.q_dot = q_dot.tolist()
        return response

# main
def main(args=None):
    rclpy.init(args=args)
    node = VelocityKinematics()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
