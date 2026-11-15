#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter

class UR3ArmController(Node):
    def __init__(self):
        super().__init__(
            'ur3_arm_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.current_joint_positions = None

        # Subscribe to /joint_states to read current robot pose
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for /joint_trajectory_controller action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected!')

    def joint_state_callback(self, msg):
        # Map current positions to the expected joint order
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == 6:
            self.current_joint_positions = positions

    def execute_pick_and_place(self):
        if self.current_joint_positions is None:
            self.get_logger().warn('Waiting for joint states from Gazebo...')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        # Point 0: Start from EXACT current position (Prevents Path Tolerance Violation)
        p0 = JointTrajectoryPoint()
        p0.positions = list(self.current_joint_positions)
        p0.velocities = [0.0] * 6
        p0.time_from_start = Duration(sec=0, nanosec=0)

        # Point 1: Move to Home / Ready Position (t = 4s)
        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        p1.velocities = [0.0] * 6
        p1.time_from_start = Duration(sec=4, nanosec=0)

        # Point 2: Reach Down to Pick (t = 8s)
        p2 = JointTrajectoryPoint()
        p2.positions = [0.5, -1.0, 1.8, -2.3, -1.57, 0.0]
        p2.velocities = [0.0] * 6
        p2.time_from_start = Duration(sec=8, nanosec=0)

        # Point 3: Lift Object (t = 11s)
        p3 = JointTrajectoryPoint()
        p3.positions = [0.5, -1.57, 1.57, -1.57, -1.57, 0.0]
        p3.velocities = [0.0] * 6
        p3.time_from_start = Duration(sec=11, nanosec=0)

        # Point 4: Place Position (t = 15s)
        p4 = JointTrajectoryPoint()
        p4.positions = [-1.2, -1.0, 1.8, -2.3, -1.57, 0.0]
        p4.velocities = [0.0] * 6
        p4.time_from_start = Duration(sec=15, nanosec=0)

        # Point 5: Return to Home (t = 19s)
        p5 = JointTrajectoryPoint()
        p5.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        p5.velocities = [0.0] * 6
        p5.time_from_start = Duration(sec=19, nanosec=0)

        goal_msg.trajectory.points = [p0, p1, p2, p3, p4, p5]

        self.get_logger().info('Sending smooth trajectory (starting from current arm pose)...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED!')
            return

        self.get_logger().info('Goal ACCEPTED! Executing smooth trajectory in Gazebo...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory SUCCEEDED! Pick and Place completed! 🎉')
        else:
            self.get_logger().error(f'Execution FAILED! Code: {result.error_code}, Message: {result.error_string}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = UR3ArmController()

    # Wait until current joint positions are received, then trigger trajectory once
    def timer_callback():
        if controller.execute_pick_and_place():
            timer.cancel()

    timer = controller.create_timer(0.5, timer_callback)
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
