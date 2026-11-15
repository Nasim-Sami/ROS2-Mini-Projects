#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class UR3AutonomousPlanner(Node):
    def __init__(self):
        super().__init__(
            'ur3_autonomous_planner',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )

        # 1. Collision Object Publisher for MoveIt scene
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # 2. Action Client for MoveIt's MoveGroup brain
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Connecting to MoveIt MoveGroup action server (/move_action)...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveIt MoveGroup connected successfully!')

    def spawn_obstacle(self):
        """Spawns the green box in MoveIt planning scene"""
        co = CollisionObject()
        co.header.frame_id = 'world'
        co.id = 'center_obstacle_box'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.12, 0.20, 0.25]

        box_pose = Pose()
        box_pose.position.x = 0.40
        box_pose.position.y = 0.0
        box_pose.position.z = 0.125
        box_pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)
        co.operation = CollisionObject.ADD

        self.get_logger().info('Publishing obstacle box to MoveIt Planning Scene...')
        self.collision_pub.publish(co)

    def plan_and_execute(self, target_joints, task_name="Goal"):
        """Sends a target pose to MoveIt to autonomously plan around obstacles"""
        self.get_logger().info(f'--- Planning collision-free path for: {task_name} ---')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4

        # Tell MoveIt to Plan AND Execute automatically
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        # Joint constraints (The destination we want to reach)
        constraints = Constraints()
        for joint_name, target_pos in target_joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_pos
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'{task_name}: Goal rejected by MoveIt!')
            return False

        self.get_logger().info(f'{task_name}: MoveIt found collision-free path! Executing...')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result

        if res.error_code.val == 1:  # 1 = SUCCESS in MoveIt error codes
            self.get_logger().info(f'{task_name}: Reached destination safely! ✅')
            return True
        else:
            self.get_logger().error(f'{task_name}: Failed with MoveIt error code: {res.error_code.val}')
            return False

def main(args=None):
    rclpy.init(args=args)
    planner = UR3AutonomousPlanner()

    # Step 1: Ensure obstacle is in the scene
    planner.spawn_obstacle()
    time.sleep(1.0)

    # Step 2: Move to Right Side (Pick Zone)
    right_pick_pose = {
        'shoulder_pan_joint': -0.8,
        'shoulder_lift_joint': -1.0,
        'elbow_joint': 1.8,
        'wrist_1_joint': -2.3,
        'wrist_2_joint': -1.57,
        'wrist_3_joint': 0.0
    }
    planner.plan_and_execute(right_pick_pose, "1. Move to Right Pick Zone")

    time.sleep(1.0)

    # Step 3: Move to Left Side (Place Zone) - MoveIt MUST navigate around obstacle!
    left_place_pose = {
        'shoulder_pan_joint': 0.8,
        'shoulder_lift_joint': -1.0,
        'elbow_joint': 1.8,
        'wrist_1_joint': -2.3,
        'wrist_2_joint': -1.57,
        'wrist_3_joint': 0.0
    }
    planner.plan_and_execute(left_place_pose, "2. Autonomous Navigation Around Obstacle to Left Place Zone")

    time.sleep(1.0)

    # Step 4: Return to Home
    home_pose = {
        'shoulder_pan_joint': 0.0,
        'shoulder_lift_joint': -1.57,
        'elbow_joint': 1.57,
        'wrist_1_joint': -1.57,
        'wrist_2_joint': -1.57,
        'wrist_3_joint': 0.0
    }
    planner.plan_and_execute(home_pose, "3. Return Home")

    self_shutdown = planner.get_logger().info('🎉 All Autonomous Tasks Completed!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
