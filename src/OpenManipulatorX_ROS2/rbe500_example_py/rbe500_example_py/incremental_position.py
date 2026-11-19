"""
cmds
colcon build --packages-select rbe500_interfaces rbe500_example_py
source install/setup.bash
ros2 run rbe500_example_py incremental_position

ros2 topic pub /joint_velocity_reference std_msgs/msg/Float64MultiArray "{data: [0.1, 0.0, 0.0, 0.0]}"
"""

from open_manipulator_msgs.srv import SetJointPosition

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
SAMPLING_TIME = 0.1

# Update joint position references from joint velocity references
class IncrementalPosition(Node):
    def __init__(self):
        super().__init__('incremental_position')
        self.q_ref = [0.0, 0.0, 0.0, 0.0]
        self.delta_q = [0.0, 0.0, 0.0, 0.0]

        # create subscription to joint velocity reference topic
        self.create_subscription(
            Float64MultiArray,
            'joint_velocity_reference',
            self.delta_q_callback,
            10)
        self.goal_client = self.create_client(
            SetJointPosition, 'goal_joint_space_path')
        self.create_timer(SAMPLING_TIME, self.update_reference)

    # Store the incoming joint velocity reference
    def delta_q_callback(self, msg):
        self.delta_q = list(msg.data)

    # Integrate one step and send the resulting joint goal
    def update_reference(self):
        self.q_ref = [
            q_ref_old + delta_q * SAMPLING_TIME
            for q_ref_old, delta_q in zip(self.q_ref, self.delta_q)
        ]

        request = SetJointPosition.Request()
        request.joint_position.joint_name = JOINT_NAMES
        request.joint_position.position = self.q_ref
        request.path_time = SAMPLING_TIME
        self.goal_client.call_async(request)

#main
def main(args=None):
    rclpy.init(args=args)
    node = IncrementalPosition()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
