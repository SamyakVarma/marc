import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import marc as robot
from threading import Thread

class PoseGoalNode(Node):
    def __init__(self):
        super().__init__("pose_goal_node")

        # Declare parameters
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", False)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")

        # Create callback group
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit2.planner_id = self.get_parameter("planner_id").value
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5



    def move_to_pose(self, x, y, z, qx, qy, qz, qw, cartesian=False):
        # Set parameters for cartesian planning
        self.moveit2.cartesian_avoid_collisions = self.get_parameter("cartesian_avoid_collisions").value
        self.moveit2.cartesian_jump_threshold = self.get_parameter("cartesian_jump_threshold").value

        # Log the target pose
        self.get_logger().info(f"Moving to pose: position=({x}, {y}, {z}), quaternion=({qx}, {qy}, {qz}, {qw})")

        # Move to pose
        self.moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=[qx, qy, qz, qw],
            cartesian=cartesian,
            cartesian_max_step=self.get_parameter("cartesian_max_step").value,
            cartesian_fraction_threshold=self.get_parameter("cartesian_fraction_threshold").value,
        )
        self.moveit2.wait_until_executed()

    def shutdown(self):
        """Shuts down the ROS 2 node and stops the executor."""
        self.executor.shutdown()
        self.executor_thread.join()
        self.destroy_node()

if __name__ == "__main__":
    rclpy.init()
    pose_goal_node = PoseGoalNode()
    try:
        # Example: Move to a specific pose
        pose_goal_node.move_to_pose(0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0, cartesian=False)
    except KeyboardInterrupt:
        pass
    finally:
        pose_goal_node.shutdown()
        rclpy.shutdown()
