"""
pick and lift: open gripper, move above the object, move down, grab it, lift it
uses the inverse_kinematics service to turn poses into joint angles

cmds (run each block in its own terminal, in this order)
terminal 1: build and start the robot controller
cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbe500_interfaces rbe500_example_py
source install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO749-if00-port0

terminal 2: start the inverse kinematics service
cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rbe500_example_py inverse_kinematics

terminal 3: run pick and lift
cd ~/PycharmProjects/WPI-RBE-500-FinalProject
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rbe500_example_py pick_and_lift
"""

import math
import time

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition

from rbe500_interfaces.srv import InverseKinematics

# object location on the chessboard (mm), measure this on the real setup
OBJECT_X = 150.0
OBJECT_Y = 00.0
OBJECT_Z = 20.0

# how far above the object the pre-grasp pose is (mm)
PRE_GRASP_HEIGHT = 80.0

# gripper pointing straight down
PITCH = math.pi / 2

# gripper positions (sent to the gripper joint)
GRIPPER_OPEN = -0.01
GRIPPER_CLOSED = 0.01  # adjust if the gripper doesn't grip the object tight enough

# time for each move (s)
PATH_TIME = 3.0

class PickAndLift(Node):
    def __init__(self):
        super().__init__('pick_and_lift')

        # create client for inverse kinematics
        self.ik_client = self.create_client(InverseKinematics, 'inverse_kinematics')

        # create client for setting joint position
        self.joint_client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        # create client for opening and closing the gripper
        self.gripper_client = self.create_client(SetJointPosition, 'goal_tool_control')

        # wait for all the services (hangs here if the controller or ik node isn't running)
        self.ik_client.wait_for_service()
        self.joint_client.wait_for_service()
        self.gripper_client.wait_for_service()

    # ask the ik service for joint angles, returns [] if the pose is unreachable
    def calculate_inverse_pose(self, x, y, z, pitch):
        # send request to inverse kinematics service
        request = InverseKinematics.Request()
        request.x = x
        request.y = y
        request.z = z
        request.pitch = pitch

        # send the request and block until the answer comes back
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        q = list(future.result().q)

        # print joint angles
        self.get_logger().info(f'pose ({x}, {y}, {z}, {pitch}) -> q = {q}')

        return q

    # move the arm to joint angles q and wait until it gets there
    def move_to(self, q):
        # send request to set joint position
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = q
        request.path_time = PATH_TIME

        future = self.joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # the service returns right away, so wait for the robot to finish moving
        time.sleep(PATH_TIME + 1.0)

    # open or close the gripper and wait until it's done
    def set_gripper(self, position):
        # send request to open or close the gripper
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['gripper']
        request.joint_position.position = [position]
        request.path_time = 1.0

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # wait for the gripper to finish moving
        time.sleep(2.0)

    # full pick and lift sequence
    def run(self):
        # calculate joint angles for the pre-grasp and grasp poses
        pre_grasp_q = self.calculate_inverse_pose(
            OBJECT_X, OBJECT_Y, OBJECT_Z + PRE_GRASP_HEIGHT, PITCH)
        grasp_q = self.calculate_inverse_pose(
            OBJECT_X, OBJECT_Y, OBJECT_Z, PITCH)

        # stop if either pose can't be reached
        if len(pre_grasp_q) == 0 or len(grasp_q) == 0:
            self.get_logger().error('object pose is not reachable')
            return

        # 0. open the gripper
        self.get_logger().info('opening gripper')
        self.set_gripper(GRIPPER_OPEN)

        # 1. move to the pre-grasp pose above the object
        self.get_logger().info('moving to pre-grasp pose')
        self.move_to(pre_grasp_q)

        # 2. move down to the grasp pose
        self.get_logger().info('moving to grasp pose')
        self.move_to(grasp_q)

        # 3. close the gripper
        self.get_logger().info('closing gripper')
        self.set_gripper(GRIPPER_CLOSED)

        # 4. move back to the pre-grasp pose with the object
        self.get_logger().info('lifting object')
        self.move_to(pre_grasp_q)

        self.get_logger().info('done')

# main
def main(args=None):
    rclpy.init(args=args)
    node = PickAndLift()

    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
