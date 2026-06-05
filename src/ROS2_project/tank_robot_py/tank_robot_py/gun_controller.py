import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Empty, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import math
import time # <--- Idő méréséhez az újratöltéshez

class GunControllerNode(Node):
    def __init__(self):
        super().__init__('gun_controller_node')
        self.br = CvBridge()

        self.g = 9.81
        self.v0 = 20.0  
        self.angle_threshold = 0.01 
        
        self.fire_enabled = False
        self.current_gun_pitch = 0.0
        self.target_pitch = 0.0
        self.has_fired = False 
        self.latest_depth_frame = None

        # --- ÚJRATÖLTÉSI (COOLDOWN) LOGIKA ---
        self.last_shot_time = 0.0
        self.reload_time = 3.0 # Másodpercben! Addig nem lő újra.

        self.sub_depth = self.create_subscription(Image, 'camera/depth_image', self.depth_callback, 10)
        self.sub_target_info = self.create_subscription(Point, '/target_info', self.target_info_callback, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.sub_fire = self.create_subscription(Bool, '/fire_enable', self.fire_enable_callback, 10)

        self.pub_gun = self.create_publisher(Float64, '/model/my_tank/joint/gun_joint/cmd_pos', 10)
        self.pub_shoot = self.create_publisher(Empty, '/shoot', 10)

        self.get_logger().info("Ágyú Ballisztikai Node Élesítve (3 mp újratöltéssel)!")

    def fire_enable_callback(self, msg):
        self.fire_enabled = msg.data

    def joint_callback(self, msg):
        if 'gun_joint' in msg.name:
            idx = msg.name.index('gun_joint')
            self.current_gun_pitch = msg.position[idx]
            
            # TŰZENGEDÉLY LOGIKA COOLDOWN-NAL
            current_time = time.time()
            if self.fire_enabled and abs(self.current_gun_pitch - self.target_pitch) < self.angle_threshold and not self.has_fired:
                if (current_time - self.last_shot_time) > self.reload_time:
                    self.get_logger().info("🎯 Teljes Lock! Tűz!")
                    self.pub_shoot.publish(Empty())
                    self.has_fired = True
                    self.last_shot_time = current_time # Lövés időpontjának mentése

    def depth_callback(self, msg):
        self.latest_depth_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def target_info_callback(self, msg):
        msg_gun_pos = Float64()

        if msg.z == 1.0 and self.latest_depth_frame is not None:
            cx, cy = int(msg.x), int(msg.y)
            height, width = self.latest_depth_frame.shape[:2]

            if 0 <= cy < height and 0 <= cx < width:
                R = self.latest_depth_frame[cy, cx]
                if math.isfinite(R) and R > 0:
                    val = (self.g * R) / (self.v0 ** 2)
                    self.target_pitch = 0.5 * math.asin(val) if val <= 1.0 else 0.3491
                    msg_gun_pos.data = self.target_pitch
                else:
                    msg_gun_pos.data = self.current_gun_pitch
                    
        elif msg.z == 0.0:
            self.has_fired = False
            msg_gun_pos.data = self.target_pitch 

        elif msg.z == -1.0:
            msg_gun_pos.data = 0.0
            self.target_pitch = 0.0
            self.has_fired = False

        msg_gun_pos.data = max(min(msg_gun_pos.data, 0.3491), -0.1745)
        self.pub_gun.publish(msg_gun_pos)

def main(args=None):
    rclpy.init(args=args)
    node = GunControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()