#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class UR3ObstaclePickAndPlace(Node):
    def __init__(self):
        super().__init__(
            'ur3_obstacle_pick_and_place',
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

        # 1. Collision Object Publisher for MoveIt RViz
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # 2. Action Client for Trajectory Execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Connecting to /joint_trajectory_controller...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected!')

    def spawn_obstacle(self):
        co = CollisionObject()
        co.header.frame_id = 'world'
        co.id = 'center_obstacle_box'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.12, 0.20, 0.25]  # Slightly smaller to give arm clearance

        box_pose = Pose()
        box_pose.position.x = 0.40  # Placed safely in front of the robot
        box_pose.position.y = 0.0
        box_pose.position.z = 0.125
        box_pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)
        co.operation = CollisionObject.ADD

        self.get_logger().info('Spawning obstacle box in MoveIt...')
        self.collision_pub.publish(co)

    def execute_pick_and_place(self):
        self.spawn_obstacle()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        # 1. Ready Position (Home) - t = 3s
        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        p1.time_from_start = Duration(sec=3, nanosec=0)

        # 2. Pick Position (Right Side) - t = 7s
        p2 = JointTrajectoryPoint()
        p2.positions = [-0.8, -1.0, 1.8, -2.3, -1.57, 0.0]
        p2.time_from_start = Duration(sec=7, nanosec=0)

        # 3. Lift Up High (Above Obstacle) - t = 11s
        p3 = JointTrajectoryPoint()
        p3.positions = [-0.8, -1.57, 1.57, -1.57, -1.57, 0.0]
        p3.time_from_start = Duration(sec=11, nanosec=0)

        # 4. Fly Over to Left Side - t = 16s
        p4 = JointTrajectoryPoint()
        p4.positions = [0.8, -1.57, 1.57, -1.57, -1.57, 0.0]
        p4.time_from_start = Duration(sec=16, nanosec=0)

        # 5. Place Position (Left Side) - t = 20s
        p5 = JointTrajectoryPoint()
        p5.positions = [0.8, -1.0, 1.8, -2.3, -1.57, 0.0]
        p5.time_from_start = Duration(sec=20, nanosec=0)

        # 6. Lift Back Up - t = 24s
        p6 = JointTrajectoryPoint()
        p6.positions = [0.8, -1.57, 1.57, -1.57, -1.57, 0.0]
        p6.time_from_start = Duration(sec=24, nanosec=0)

        # 7. Return to Home - t = 28s
        p7 = JointTrajectoryPoint()
        p7.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        p7.time_from_start = Duration(sec=28, nanosec=0)

        goal_msg.trajectory.points = [p1, p2, p3, p4, p5, p6, p7]

        self.get_logger().info('Sending single 28-second trajectory...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal REJECTED!')
            return

        self.get_logger().info('Trajectory goal ACCEPTED! Watch robot arm move...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('🎉 Pick and Place completed successfully!')
        else:
            self.get_logger().error(f'Failed with code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = UR3ObstaclePickAndPlace()
    
    # Send goal EXACTLY ONCE after 0.5s connection setup (No repeating timer bug!)
    def one_shot_callback():
        one_shot_timer.cancel()
        controller.execute_pick_and_place()

    one_shot_timer = controller.create_timer(0.5, one_shot_callback)
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
