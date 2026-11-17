"""
inverse kinematics node for the OpenManipulator-X
- service /inverse_kinematics: pose -> joint angles (does not move the arm)
- topic /calculate_inverse_pose: pose -> joint angles, then moves the arm

units: x, y, z in mm, pitch and joint angles in radians
pitch = 0 is gripper level, pitch = pi/2 is gripper pointing straight down

cmds
build and run the node
colcon build --packages-select rbe500_interfaces rbe500_example_py
source install/setup.bash
ros2 run rbe500_example_py inverse_kinematics

calculate only (returns q), data is x, y, z, pitch
ros2 service call /inverse_kinematics rbe500_interfaces/srv/InverseKinematics "{x: 281.4, y: 0.0, z: 224.326, pitch: 0.0}"
ros2 service call /inverse_kinematics rbe500_interfaces/srv/InverseKinematics "{x: 200.0, y: 50.0, z: 50.0, pitch: 1.5708}"

calculate and move the arm, data is [x, y, z, pitch]
ros2 topic pub --once /calculate_inverse_pose std_msgs/msg/Float32MultiArray "{data: [281.4, 0.0, 224.326, 0.0]}"
"""

import math

import rclpy
from rclpy.node import Node
from rbe500_interfaces.srv import InverseKinematics
from open_manipulator_msgs.srv import SetJointPosition

from std_msgs.msg import Float32MultiArray

# length constants (mm)
L1 = 96.326                     # base to joint 2 (height)
L2 = math.sqrt(128**2 + 24**2)  # link 2 has a 24 mm offset
L3 = 124                        # joint 3 to joint 4
L4 = 133.4                      # joint 4 to gripper tip

# angle offset on joint 2 because of the 24 mm offset
OFFSET = math.atan2(128, 24)

# calculate joint angles from the end effector pose
# returns [q1, q2, q3, q4] in radians, or None if the pose can't be reached
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

    # now solve the 2 link arm (L2, L3) that reaches the wrist point
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
    q4 = pitch - q2 - q3  # q2 + q3 + q4 must add up to the gripper pitch

    return [q1, q2, q3, q4]

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # create service for end effector pose -> joint positions
        self.ik_service = self.create_service(
            InverseKinematics,
            'inverse_kinematics',
            self.inverse_kinematics_callback)

        # create subscription for calculating inverse kinematics and moving the arm
        self.ik_subscription = self.create_subscription(
            Float32MultiArray,
            'calculate_inverse_pose',
            self.calculate_inverse_pose_callback,
            10)

        # create client for setting joint position
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

    # topic callback: calculate joint angles and move the arm there
    def calculate_inverse_pose_callback(self, msg):
        # message pose
        x, y, z, pitch = msg.data

        q = inverse_kinematics(x, y, z, pitch)

        if q is None:
            self.get_logger().error(f'pose ({x}, {y}, {z}, {pitch}) is not reachable')
            return

        # print joint positions
        self.get_logger().info(f'q = {q}')

        # send request to set joint position
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = q
        request.path_time = 5.0

        # don't wait for the result here, the arm just moves on its own
        self.future = self.client.call_async(request)

    # service callback: calculate joint angles and send them back (no motion)
    def inverse_kinematics_callback(self, request, response):
        # request pose
        x, y, z, pitch = request.x, request.y, request.z, request.pitch

        q = inverse_kinematics(x, y, z, pitch)

        if q is None:
            self.get_logger().error(f'pose ({x}, {y}, {z}, {pitch}) is not reachable')
            response.q = []  # empty list tells the caller it's unreachable
            return response

        # print joint positions
        self.get_logger().info(f'q = {q}')

        response.q = q
        return response

# main
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()

    # keep the node running so it can answer requests
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
