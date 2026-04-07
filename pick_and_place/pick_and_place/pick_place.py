#!/usr/bin/env python3
"""
Mitsubishi RV-2FR Pick and Place Node
pymoveit2 compatible – NO wrist spinning
"""

from threading import Thread
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface


# ---------------- RV-2FR CONSTANTS ----------------
class RV2FR:
    joint_names = [
        "rv2fr_joint_1", "rv2fr_joint_2", "rv2fr_joint_3",
        "rv2fr_joint_4", "rv2fr_joint_5", "rv2fr_joint_6"
    ]

    base_link_name = "rv2fr_base"
    end_effector_name = "rv2fr_default_tcp"
    MOVE_GROUP_ARM = "rv2fr"

    gripper_joint_names = ["finger_joint_l", "finger_joint_r"]
    MOVE_GROUP_GRIPPER = "gripper"

    OPEN_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]
    CLOSED_GRIPPER_JOINT_POSITIONS = [-0.20, 0.20]


# ---------------- MAIN NODE ----------------
class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        self.declare_parameter("target_color", "B")
        self.target_color = self.get_parameter("target_color").value.upper()

        self.already_moved = False
        self.callback_group = ReentrantCallbackGroup()

        # MoveIt2 arm
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=RV2FR.joint_names,
            base_link_name=RV2FR.base_link_name,
            end_effector_name=RV2FR.end_effector_name,
            group_name=RV2FR.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Motion limits
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Cartesian tuning (IMPORTANT)
        self.moveit2.cartesian_step = 0.005
        self.moveit2.cartesian_jump_threshold = 0.0

        # Gripper
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=RV2FR.gripper_joint_names,
            open_gripper_joint_positions=RV2FR.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=RV2FR.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=RV2FR.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_controller/joint_trajectory",
        )

        # Subscriber
        self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )

        self.get_logger().info(
            f"Waiting for target color {self.target_color}"
        )

        # Joint configurations
        self.start_joints = [
            0.0, 0.0, math.radians(90.0),
            0.0, math.radians(90.0), 0.0
        ]

        self.home_joints = self.start_joints

        self.drop_joints = [
            math.radians(-90.0),
            math.radians(60.0),
            math.radians(75.0),
            0.0,
            math.radians(45.0),
            0.0,
        ]

        # Go to start
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

    # ---------------- CALLBACK ----------------
    def coords_callback(self, msg):
        if self.already_moved:
            return

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id != self.target_color:
                return

            self.already_moved = True

            x = float(x)+0.02
            y = float(y)
            z = float(z)

            self.get_logger().info(
                f"Target locked at x={x}, y={y}, z={z}"
            )

            # ---------------- PICK SEQUENCE ----------------

            quat_xyzw = [1.0, 0.0, 0.0, 0.0]   # top-down
            above_pick = [x-0.02, y, z + 0.30]
            down_pick  = [x-0.02, y, z + 0.10]     # straight down
            lift_pick  = [x-0.02, y, z + 0.30]

            # Home
            self.gripper.open()
            self.gripper.wait_until_executed()
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # Move ABOVE object (IK)
            self.moveit2.move_to_pose(
                position=above_pick,
                quat_xyzw=quat_xyzw
            )
            self.moveit2.wait_until_executed()

            # -------- CARTESIAN DOWN (NO WRIST SPIN) --------
            self.moveit2.move_to_pose(
                position=down_pick,
                quat_xyzw=quat_xyzw,
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # Close gripper
            self.gripper.close()
            self.gripper.wait_until_executed()

            # -------- CARTESIAN UP --------
            self.moveit2.move_to_pose(
                position=lift_pick,
                quat_xyzw=quat_xyzw,
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # Drop
            self.moveit2.move_to_configuration(self.drop_joints)
            self.moveit2.wait_until_executed()

            self.gripper.open()
            self.gripper.wait_until_executed()

            # Return
            self.moveit2.move_to_configuration(self.start_joints)
            self.moveit2.wait_until_executed()

            self.get_logger().info("Pick and place completed successfully.")
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


# ---------------- MAIN ----------------
def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
