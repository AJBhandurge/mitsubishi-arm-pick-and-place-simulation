#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf_transformations

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        #Subscribe to the camera feed for color detection
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)
        
        #Publisher for the detected color coordinates to be used by the pick and place node.
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)

        # OpenCV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # TF2 setup to get the camera's pose relative to the robot base.
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Intrinsic parameters calculated from URDF (FOV 1.0, Width 640)
        self.fx = 585.9  
        self.fy = 585.9
        self.cx = 320.0  
        self.cy = 160.0  

        self.get_logger().info("Color Detector Node Started - Top-Down Projection Mode")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Standard HSV Ranges
        color_ranges = {
            "R": [(0, 120, 70), (10, 255, 255)],
            "G": [(50, 100, 100), (75, 255, 255)],
            "B": [(100, 150, 0), (140, 255, 255)]
        }

        for color_id, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 100:
                    x_pix, y_pix, w, h = cv2.boundingRect(cnt)
                    u, v = x_pix + w // 2, y_pix + h // 2

                    try:
                        # 1. Lookup the camera's pose relative to the robot base
                        # Using camera_link_optical because your URDF defines the axes correctly there
                        t = self.tf_buffer.lookup_transform(
                            "rv2fr_base", 
                            "camera_link_optical", 
                            rclpy.time.Time(),
                            timeout=Duration(seconds=0.1))

                        # 2. Get the vertical distance (Z) from camera to tabletop
                        # Translation.z is the camera height (~1.0m) minus object height (~0.05m)
                        Z_dist = t.transform.translation.z - 0.05

                        # 3. Calculate pixel displacement in meters
                        # This finds how far 'Left/Right' and 'Up/Down' the object is from the center of the image
                        X_disp = (u - self.cx) * Z_dist / self.fx
                        Y_disp = ((v - self.cy) * Z_dist / self.fy)
                        
                        # 4. Create the point in the local Camera Optical frame
                        # We use 0.0 for the local Z because we want to find the position ON the table,
                        # not 1 meter past the table.
                        pt_optical = np.array([X_disp, Y_disp, 0.0, 1.0])

                        # 5. Build the 4x4 Transformation Matrix from TF
                        q = [t.transform.rotation.x, t.transform.rotation.y, 
                             t.transform.rotation.z, t.transform.rotation.w]
                        T = tf_transformations.quaternion_matrix(q)
                        T[:3, 3] = [t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z]

                        # 6. Transform to Base Frame
                        # pt_base[0] = Camera_X (0.35) + displacement
                        pt_base = T @ pt_optical

                        # 7. Final Coordinates
                        msg_str = f"{color_id},{pt_base[0]:.3f},{pt_base[1]:.3f},0.05"
                        self.coords_pub.publish(String(data=msg_str))

                        # Visualize on screen
                        cv2.rectangle(frame, (x_pix, y_pix), (x_pix + w, y_pix + h), (0, 255, 0), 2)
                        cv2.putText(frame, f"{color_id}: {pt_base[0]:.2f}-{0.03}, {pt_base[1]:.2f}", 
                                    (x_pix, y_pix - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    except Exception as e:
                        continue

        cv2.imshow("Detection Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()