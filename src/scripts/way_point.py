#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import marc as robot
from scipy.spatial.transform import Rotation as R

def main():
    rclpy.init()
    node = Node("arm_waypoint_follower")

    # Define waypoints as a list of [position, orientation] pairs
    waypoints = [
       #quad
        ([0.09, 0.05, 0.00], [0.0, 1.57, 0.1]),
        ([0.14, 0.05 ,0.0], [0.0, 1.57, 0.1]),
        ([0.14, -0.05, 0.0], [0.0, 1.57, 0.1]),
        ([0.09, -0.05, 0.00], [0.0, 1.57, 0.1]),
        ([0.09, 0.05, 0.00], [0.0, 1.57, 0.1])
    ]

    # Convert euler angles to quaternions for each waypoint
    waypoints_quat = []
    for position, euler in waypoints:
        r = R.from_euler('xyz', euler)
        quat = r.as_quat()  # Returns [x, y, z, w]
        waypoints_quat.append((position, quat))

    # Declare parameters
    node.declare_parameter("synchronous", True)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    # Create callback group
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.wait_until_executed()
    # Get parameters
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    cartesian_fraction_threshold = node.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
    cartesian_jump_threshold = node.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
    cartesian_avoid_collisions = node.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value

    # Set planner parameters
    moveit2.planner_id = node.get_parameter("planner_id").get_parameter_value().string_value
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Set movement speed
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Follow waypoints
    for i, (position, quat_xyzw) in enumerate(waypoints_quat):
        node.get_logger().info(
            f"Moving to waypoint {i+1}/{len(waypoints_quat)}: "
            f"{{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )

        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        moveit2.wait_until_executed()
        
        node.get_logger().info(f"Completed waypoint {i+1}/{len(waypoints_quat)}")
        node.create_rate(2.0).sleep()  # 0.5 second pause

    node.get_logger().info("All waypoints completed")
    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()