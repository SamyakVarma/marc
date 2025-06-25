#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import marc as robot
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from queue import Queue
#home 0.024,0.0,0.111 quat=0,0,0,1
def main():
    rclpy.init()
    node = Node("arm_pose_goal")
    node.declare_parameter("position", [0.10, 0.0, 0.03])
    # Specify the desired orientation in roll, pitch, yaw (in radians)
    roll, pitch, yaw = 0.00, 0.0, 0.0
    r = R.from_euler('xyz', [roll, pitch, yaw])
    quaternion = r.as_quat()  # Returns [x, y, z, w]
    node.declare_parameter("quat_xyzw", quaternion.tolist())
    node.declare_parameter("synchronous", True)
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    # Declare parameters for cartesian planning
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
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

    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    # Create a queue to store coordinates
    coord_queue = Queue()

    # Create subscriber before starting the executor
    def click_callback(msg):
        # Convert centimeters to meters and round to 2 decimal places
        x_meters = round(msg.x / 100.0, 2)
        y_meters = round(msg.y / 100.0, 2)
        z_meters = round(msg.z / 100.0, 2)
        
        node.get_logger().info(
            f'Received coordinates (cm): x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
        )
        node.get_logger().info(
            f'Converted to meters: x={x_meters}, y={y_meters}, z={z_meters}'
        )
        
        # Push coordinates to queue in meters (rounded)
        coord_queue.put([x_meters, y_meters, z_meters])
        
    click_sub = node.create_subscription(
        Point,
        'key_xyz',
        click_callback,
        10
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions")
        .get_parameter_value()
        .bool_value
    )

    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Main loop to continuously process coordinates
    rate = node.create_rate(10)  # 10Hz refresh rate
    try:
        while rclpy.ok():
            if not coord_queue.empty():
                # Get next position from queue
                position = coord_queue.get()
                
                # Move to pose
                node.get_logger().info(
                    f"Moving to position: {list(position)}, quat_xyzw: {list(quat_xyzw)}"
                )
                
                result = moveit2.move_to_pose(
                    position=position,
                    quat_xyzw=quat_xyzw,
                    cartesian=cartesian,
                    cartesian_max_step=cartesian_max_step,
                    cartesian_fraction_threshold=cartesian_fraction_threshold,
                )
                
                # Wait for movement to complete and check result
                if result:
                    state = moveit2.wait_until_executed()
            
            rate.sleep()

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()
        executor_thread.join()
        exit(0)


if __name__ == "__main__":
    main()
