import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene

class MelfaSorter(Node):
    def __init__(self):
        super().__init__('melfa_sorter')
        
        # Publishers for world and attached objects
        self.scene_pub = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, 'attached_collision_object', 10)
        
        # URDF link names (verify these match your URDF)
        self.base_frame = "world"
        self.flange_link = "rv7frl_hand_flange"
        self.wrist_link = "rv7frl_wrist"

    def spawn_box(self, box_id, x, y, z):
        """ Adds a new collision object to the scene """
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = box_id
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]
        
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        self.scene_pub.publish(obj)
        self.get_logger().info(f"Spawned {box_id} at {x}, {y}, {z}")

    def attach_box(self, box_id):
        """ 
        Attaches the box to the robot. 
        Crucial: touch_links stops the 'Purple Box' from causing a collision error.
        """
        aco = AttachedCollisionObject()
        aco.link_name = self.flange_link
        aco.object.id = box_id
        aco.object.header.frame_id = self.flange_link
        
        # List links that are ALLOWED to touch the box
        aco.touch_links = [self.flange_link, self.wrist_link, "rv7frl_forearm"]
        
        # Set pose relative to the flange (centered on flange)
        pose = Pose()
        pose.orientation.w = 1.0
        aco.object.primitive_poses.append(pose)
        aco.object.operation = CollisionObject.ADD
        
        self.attached_pub.publish(aco)
        self.get_logger().info(f"Attached {box_id} (Purple status active)")

    def run_sorting(self):
        # 1. Spawn a box further away to avoid initial collision
        self.spawn_box("block_1", 0.5, 0.0, 0.1)
        
        # 2. Logic to move (Requires MoveGroupInterface to EXECUTE)
        # Note: In Python ROS 2 Humble, you usually call:
        # success, plan, _, _ = move_group.plan()
        # if success: move_group.execute(plan)
        
        # 3. Attach when robot is at the location
        self.get_logger().info("Simulation: Attaching object...")
        self.attach_box("block_1")

def main():
    rclpy.init()
    node = MelfaSorter()
    node.run_sorting()
    # Keep node alive to maintain the scene
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()