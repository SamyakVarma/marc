#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import marc as robot
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool

def main():
    rclpy.init()
    node = Node("pick_drop_sequence")

    # Define pick and drop positions with orientations
    pick_position = [0.12, 0.09, -0.007]  # Adjust these coordinates
    pick_orientation = [0.0, 0.0, 0.0]    # [roll, pitch, yaw] in radians
    
    drop_position = [0.05, 0.15, 0.13]     # Adjust these coordinates
    drop_orientation = [0.0, 0.0, 0.0]    # [roll, pitch, yaw] in radians

    # Convert orientations to quaternions
    r_pick = R.from_euler('xyz', pick_orientation)
    pick_quat = r_pick.as_quat()
    r_drop = R.from_euler('xyz', drop_orientation)
    drop_quat = r_drop.as_quat()

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

    # Create publisher for end effector state
    end_effector_pub = node.create_publisher(
        Bool,
        '/end_state',
        10
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Set movement speed
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Create end effector message
    ee_msg = Bool()

    # Sequence: Pick and Drop
    try:
        # 1. Open end effector
        node.get_logger().info("Opening end effector")
        ee_msg.data = True
        end_effector_pub.publish(ee_msg)
        node.create_rate(0.5).sleep()  # Wait for end effector to open

        # 2. Move to pick position
        node.get_logger().info("Moving to pick position")
        moveit2.move_to_pose(
            position=pick_position,
            quat_xyzw=pick_quat,
            cartesian=False,
            cartesian_max_step=0.0025,
        )
        moveit2.wait_until_executed()

        # 3. Close end effector (grab object)
        node.get_logger().info("Closing end effector to grab object")
        ee_msg.data = False
        end_effector_pub.publish(ee_msg)
        node.create_rate(0.5).sleep()  # Wait for end effector to close


        # 5. Move to drop position
        node.get_logger().info("Moving to drop position")
        moveit2.move_to_pose(
            position=drop_position,
            quat_xyzw=drop_quat,
            cartesian=False,
            cartesian_max_step=0.0025,
        )
        moveit2.wait_until_executed()

        # 6. Open end effector (release object)
        node.get_logger().info("Opening end effector to release object")
        ee_msg.data = True
        end_effector_pub.publish(ee_msg)
        node.create_rate(0.5).sleep()  # Wait for end effector to open

        node.get_logger().info("Pick and drop sequence completed")

    except Exception as e:
        node.get_logger().error(f"An error occurred: {str(e)}")
    finally:
        rclpy.shutdown()
        executor_thread.join()
        exit(0)

if __name__ == "__main__":
    main()