"""
Forward Kinematics node for the OpenManipulator-X.

What it does:
  1. Subscribes to /joint_states to get the current joint angles.
  2. Builds one Denavit-Hartenberg (DH) matrix per joint and multiplies them
     together to get the end-effector pose in the base frame.
  3. Publishes that pose on /end_effector_pose (all lengths in millimeters).

Run it with:   ros2 run omx_kinematics forward_kinematics
Check it with: ros2 topic echo /end_effector_pose
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


# ---------------------------------------------------------------------------
# Robot dimensions from the assignment figure (millimeters)
# ---------------------------------------------------------------------------
BASE_HEIGHT = 96.326   # base frame (bottom of robot) up to the joint 2 axis
LINK2_UP = 128.0       # joint 2 -> joint 3, vertical part
LINK2_FORWARD = 24.0   # joint 2 -> joint 3, small forward part
LINK3 = 124.0          # joint 3 -> joint 4
LINK4 = 133.4          # joint 4 -> end effector (between the gripper fingers)

# Link 2 is not straight (it is an "L" shape), so treat it as one straight
# line from joint 2 to joint 3. Its length comes from Pythagoras...
LINK2 = math.sqrt(LINK2_UP**2 + LINK2_FORWARD**2)   # about 130.23 mm
# ...and this is the angle between that straight line and the vertical part.
# We need it so that joint angle 0 matches the real robot's zero position.
LINK2_ANGLE = math.atan2(LINK2_UP, LINK2_FORWARD)   # about 1.385 rad (79.4 deg)


def dh_matrix(theta, d, a, alpha):
    """Standard DH homogeneous transform from frame i-1 to frame i."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def forward_kinematics(q1, q2, q3, q4):
    """Return the 4x4 transform from the base frame to the end effector.

    DH table (theta, d, a, alpha):
      Link 1: q1,               BASE_HEIGHT, 0,     -90 deg
      Link 2: q2 - LINK2_ANGLE, 0,           LINK2,   0
      Link 3: q3 + LINK2_ANGLE, 0,           LINK3,   0
      Link 4: q4,               0,           LINK4,   0
    """
    T1 = dh_matrix(q1, BASE_HEIGHT, 0.0, -math.pi / 2)
    T2 = dh_matrix(q2 - LINK2_ANGLE, 0.0, LINK2, 0.0)
    T3 = dh_matrix(q3 + LINK2_ANGLE, 0.0, LINK3, 0.0)
    T4 = dh_matrix(q4, 0.0, LINK4, 0.0)

    # Chain the transforms together: base -> 1 -> 2 -> 3 -> end effector
    return T1 @ T2 @ T3 @ T4


def rotation_to_quaternion(R):
    """Convert a 3x3 rotation matrix into a quaternion (x, y, z, w)."""
    w = 0.5 * math.sqrt(max(0.0, 1 + R[0][0] + R[1][1] + R[2][2]))
    x = 0.5 * math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2]))
    y = 0.5 * math.sqrt(max(0.0, 1 - R[0][0] + R[1][1] - R[2][2]))
    z = 0.5 * math.sqrt(max(0.0, 1 - R[0][0] - R[1][1] + R[2][2]))
    # The square roots only give the size of x, y, z; these fix their signs
    x = math.copysign(x, R[2][1] - R[1][2])
    y = math.copysign(y, R[0][2] - R[2][0])
    z = math.copysign(z, R[1][0] - R[0][1])
    return x, y, z, w


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics')

        # Listen to the joint angles coming from the robot controller
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Publish the calculated end-effector pose
        self.publisher = self.create_publisher(Pose, 'end_effector_pose', 10)

    def joint_state_callback(self, msg):
        # Look up each joint by name, since the order in the message is not guaranteed
        q1 = msg.position[msg.name.index('joint1')]
        q2 = msg.position[msg.name.index('joint2')]
        q3 = msg.position[msg.name.index('joint3')]
        q4 = msg.position[msg.name.index('joint4')]

        T = forward_kinematics(q1, q2, q3, q4)

        # Fill in the Pose message: position is the last column of T,
        # orientation comes from the top-left 3x3 rotation part of T
        pose = Pose()
        pose.position.x = T[0][3]
        pose.position.y = T[1][3]
        pose.position.z = T[2][3]
        qx, qy, qz, qw = rotation_to_quaternion(T[0:3, 0:3])
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
