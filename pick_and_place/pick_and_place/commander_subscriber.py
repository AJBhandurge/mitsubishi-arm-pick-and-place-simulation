#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "rv2fr_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        
        arm_controller.joint_names = ["rv2fr_joint_1", "rv2fr_joint_2", "rv2fr_joint_3", "panda_joint4", "rv2fr_joint_5", "rv2fr_joint_6"]
        gripper_controller.joint_names = ["finger_joint_l", "finger_joint_r"]
        
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        
        arm_goal.positions = msg.position[:6]
        gripper_goal.positions = [msg.position[6]]
        
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main():
    rclpy.init()
    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()