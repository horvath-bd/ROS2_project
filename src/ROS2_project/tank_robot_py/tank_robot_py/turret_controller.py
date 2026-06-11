import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import math
import time

from ultralytics import YOLO

class TurretControllerNode(Node):
    def __init__(self):
        super().__init__('turret_controller_node')
        self.br = CvBridge()
        
        # --- YAML WAYPOINT KEZELÉS ---
        self.yaml_path = '/home/horvathbd/ros_project_ws/src/ROS2_project/tank_robot/config/waypoints.yaml'
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.load_waypoints()

        # --- TF2 INICIALIZÁLÁSA ---
        from tf2_ros import TransformException
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- PUBLISHER A NAV2-NEK ---
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # --- FELIRATKOZÁSOK ---
        self.sub_image = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        self.sub_joints = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.sub_combat = self.create_subscription(Bool, '/combat_status', self.combat_status_callback, 10)
        
        self.pub_turret = self.create_publisher(Float64, '/model/my_tank/joint/turret_joint/cmd_pos', 10)
        self.pub_target_info = self.create_publisher(Point, '/target_info', 10)
        self.pub_fire_enable = self.create_publisher(Bool, '/fire_enable', 10)

        self.nav_cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # --- ÁLLAPOTVÁLTOZÓK ---
        self.combat_mode = False
        self.nav_canceled = False
        self.lock_time = 0.0         
        self.target_lost_counter = 0 
        self.current_turret_yaw = 0.0
        self.kp_vision = 0.0015  

        self.get_logger().info("Betöltés: Saját YOLOv8 modell...")
        self.model = YOLO('/home/horvathbd/ros_project_ws/Model_weights/best.pt') 
        self.get_logger().info("Saját YOLOv8 Harci Rendszer Élesítve (Hiszterézissel és fék-késleltetéssel)!")

        # Elindítjuk az első waypointot 15 másodperces biztonsági késleltetéssel
        self.timer = self.create_timer(15.0, self.send_initial_goal)
        
        # Távolságellenőrzés a térképen
        self.location_timer = self.create_timer(0.1, self.check_distance_on_map)

        # Figyeljük, hol vagyunk éppen (ezt fogjuk lementeni)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        # Ezen fogjuk visszanyomni a Nav2-nek a helyes pozíciót
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.saved_pose = None

    def load_waypoints(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']
            self.get_logger().info(f"📋 Sikeresen betöltve {len(self.waypoints)} waypoint a YAML-ból!")
        except Exception as e:
            self.get_logger().error(f"❌ Nem sikerült beolvasni a YAML fájlt: {e}")

    def send_initial_goal(self):
        self.timer.cancel()
        self.send_current_waypoint()

    def send_current_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("🏁 Minden pontot elértünk, misszió sikeresen teljesítve!")
            return

        wp = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(f"📍 Nav2 Cél küldése (Index: {self.current_waypoint_idx}) -> X: {wp['x']}, Y: {wp['y']}")
        
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.pub_nav_goal.publish(goal)

    def check_distance_on_map(self):
        if self.combat_mode or self.current_waypoint_idx >= len(self.waypoints):
            return

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            my_x = t.transform.translation.x
            my_y = t.transform.translation.y
            
            wp = self.waypoints[self.current_waypoint_idx]
            distance = math.sqrt((wp['x'] - my_x)**2 + (wp['y'] - my_y)**2)
            
            if distance < 0.5:
                self.get_logger().warn(f"✅ MAP ALAPÚ IGAZOLÁS: Waypoint {self.current_waypoint_idx} elérve! Léptetés...")
                self.current_waypoint_idx += 1
                self.send_current_waypoint()
                
        except Exception:
            pass

    def amcl_callback(self, msg):
        # Csak akkor frissítjük a mentett pózt, ha ÉPPEN NEM harcolunk
        if not self.combat_mode:
            self.saved_pose = msg

    def combat_status_callback(self, msg):
        if msg.data == False and self.combat_mode:
            self.get_logger().error("⚔️ AZ ELLENSÉG MEGHALT! Treshold visszaállítva 80%-ra, navigáció újraindul...")
            self.combat_mode = False
            self.nav_canceled = False 
            self.target_lost_counter = 0 
            
            # --- AMCL RELOKALIZÁCIÓ ---
            if self.saved_pose is not None:
                self.get_logger().info("🗺️ AMCL Relokalizáció: Visszaállítom a harc előtti biztos pozíciót!")
                self.saved_pose.header.stamp = self.get_clock().now().to_msg()
                self.pub_initial_pose.publish(self.saved_pose)
                time.sleep(1.0) # Várjunk 1 másodpercet, amíg az AMCL feldolgozza!
                
            self.send_current_waypoint()

    def cancel_navigation(self):
        if not self.nav_cancel_client.wait_for_service(timeout_sec=1.0):
            return
        req = CancelGoal.Request()
        self.nav_cancel_client.call_async(req)
        self.get_logger().info("🛑 ELLENSÉG BEMÉRVE ÉS MEGERŐSÍTVE! Navigáció leállítva!")

    def image_callback(self, msg):
        cv_image = self.br.compressed_imgmsg_to_2cv(msg, desired_encoding='bgr8') if hasattr(self.br, 'compressed_imgmsg_to_2cv') else self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = cv_image.shape[:2]
        center_x = width // 2

        # --- HISZTERÉZIS LOGIKA ---
        current_threshold = 0.70 if self.combat_mode else 0.80

        # YOLO meghívása az aktuális, dinamikus küszöbértékkel
        results = self.model(cv_image, conf=current_threshold, imgsz=384, device=0, verbose=False)
        target_visible = False
        cx, cy = 0, 0
        best_box_area = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                
                area = (x2 - x1) * (y2 - y1)
                if area > best_box_area:
                    best_box_area = area
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    target_visible = True
                    
                    # Rajzolás a képernyőre
                    cv_color = (0, 165, 255) if self.combat_mode else (0, 0, 255) 
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), cv_color, 2)
                    cv2.circle(cv_image, (cx, cy), 5, cv_color, -1)
                    cv2.putText(cv_image, f"TARGET {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 2)

        cv2.drawMarker(cv_image, (center_x, height // 2), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        info_msg = Point()
        fire_msg = Bool()

        if target_visible:
            self.target_lost_counter = 0 
            
            # 1. SZAKASZ: Ellenség meglátva, torony indítása, időzítő indul!
            if not self.combat_mode:
                self.combat_mode = True
                self.nav_canceled = False
                self.lock_time = time.time()
                self.get_logger().warn("👀 Célpont 80% felett! Torony rááll, érzékenység lejjebb véve 70%-ra...")

            # 2. SZAKASZ: 0.5 másodperc letelt, navigáció satufék!
            if self.combat_mode and not self.nav_canceled:
                if time.time() - self.lock_time >= 0.5:
                    self.cancel_navigation()
                    self.nav_canceled = True

            error_x = center_x - cx
            self.pub_turret.publish(Float64(data=self.current_turret_yaw + (error_x * self.kp_vision)))
            info_msg.x, info_msg.y = float(cx), float(cy)
            
            if abs(error_x) <= 4:
                info_msg.z = 1.0
                fire_msg.data = True
                cv2.putText(cv_image, "LOCKED - FIRE ENABLED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                info_msg.z = 0.0
                fire_msg.data = False
                cv2.putText(cv_image, "ALIGNING...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            if self.combat_mode:
                self.target_lost_counter += 1
                if self.target_lost_counter > 15: 
                    self.get_logger().warn("👻 Vaklárma vagy elvesztett cél! Harci mód megszakítva, érzékenység vissza 80%-ra...")
                    self.combat_mode = False
                    self.nav_canceled = False
                    self.target_lost_counter = 0
                    
                    # --- AMCL RELOKALIZÁCIÓ ITT IS! ---
                    if self.saved_pose is not None:
                        self.get_logger().info("🗺️ AMCL Relokalizáció: Visszaállítom a harc előtti biztos pozíciót!")
                        self.saved_pose.header.stamp = self.get_clock().now().to_msg()
                        self.pub_initial_pose.publish(self.saved_pose)
                        time.sleep(1.0)
                        
                    self.send_current_waypoint() 

            if not self.combat_mode:
                self.pub_turret.publish(Float64(data=0.0))
                cv2.putText(cv_image, f"NAVIGATING TO WP {self.current_waypoint_idx}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            info_msg.x, info_msg.y, info_msg.z = -1.0, -1.0, -1.0
            fire_msg.data = False

        self.pub_target_info.publish(info_msg)
        self.pub_fire_enable.publish(fire_msg)
        cv2.imshow("YOLOv8 Turret View", cv_image)
        cv2.waitKey(1)

    def joint_callback(self, msg):
        if 'turret_joint' in msg.name:
            self.current_turret_yaw = msg.position[msg.name.index('turret_joint')]

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TurretControllerNode())
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()