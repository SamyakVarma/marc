
import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import marc as robot
from scipy.spatial.transform import Rotation as R

def go_to_pose(moveit2, x, y, z, roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw])
    quat_xyzw = r.as_quat()

    # Move to pose
    result = moveit2.move_to_pose(
        position=[x, y, z],
        quat_xyzw=quat_xyzw,
        cartesian=False,
        cartesian_max_step=0.0025,
        cartesian_fraction_threshold=0.0
    )

    if result:
        moveit2.wait_until_executed()
        return True
    else:
        return False

def create_moveit2_interface(node):
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
    )
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0
    return moveit2
