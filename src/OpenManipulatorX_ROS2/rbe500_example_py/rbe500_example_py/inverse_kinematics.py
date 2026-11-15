"""
cmds
colcon build --packages-select rbe500_interfaces rbe500_example_py
source install/setup.bash
ros2 run rbe500_example_py inverse_kinematics

ros2 service call /inverse_kinematics rbe500_interfaces/srv/InverseKinematics "{x: 281.4, y: 0.0, z: 224.326, pitch: 0.0}"
ros2 service call /inverse_kinematics rbe500_interfaces/srv/InverseKinematics "{x: 200.0, y: 100.0, z: 50.0, pitch: 1.5708}"
"""

import math

import rclpy
from rclpy.node import Node
from rbe500_interfaces.srv import InverseKinematics

# length constants (mm)
L1 = 96.326
L2 = math.sqrt(128**2 + 24**2)  # link 2 has a 24 mm offset
L3 = 124
L4 = 133.4

# angle offset on joint 2 because of the 24 mm offset
OFFSET = math.atan2(128, 24)

# calculate joint angles from the end effector pose
def inverse_kinematics(x, y, z, pitch):
    # joint 1 just points the arm at the target
    q1 = math.atan2(y, x)

    # the rest is a 2D problem in the plane of the arm
    # r = distance out from the base, s = height above joint 2
    r = math.sqrt(x**2 + y**2)
    s = z - L1

    # step back from the end effector along the gripper to get the wrist (joint 4)
    r_w = r - L4 * math.cos(pitch)
    s_w = s + L4 * math.sin(pitch)

    # law of cosines for the elbow angle
    D = (r_w**2 + s_w**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if abs(D) > 1:
        return None  # too far away to reach

    gamma = math.acos(D)  # elbow angle (elbow up)

    # angle of link 2 and link 3 measured up from horizontal
    theta2 = math.atan2(s_w, r_w) + math.atan2(L3 * math.sin(gamma), L2 + L3 * math.cos(gamma))
    theta3 = theta2 - gamma

    # convert to the robot's joint angles (positive joints tilt the arm down)
    q2 = OFFSET - theta2
    q3 = -theta3 - q2
    q4 = pitch - q2 - q3

    return [q1, q2, q3, q4]

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # create service for end effector pose -> joint positions
        self.ik_service = self.create_service(
            InverseKinematics,
            'inverse_kinematics',
            self.inverse_kinematics_callback)

    def inverse_kinematics_callback(self, request, response):
        # request pose
        x, y, z, pitch = request.x, request.y, request.z, request.pitch

        q = inverse_kinematics(x, y, z, pitch)

        if q is None:
            self.get_logger().error(f'pose ({x}, {y}, {z}, {pitch}) is not reachable')
            response.q = []
            return response

        # print joint positions
        self.get_logger().info(f'q = {q}')

        response.q = q
        return response

# main
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
