import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import time
import math

class ShootControllerNode(Node):
    def __init__(self):
        super().__init__('shoot_controller_node')
        
        self.sub_shoot = self.create_subscription(Empty, '/shoot', self.shoot_callback, 10)
        # Feliratkozunk az odometriára, hogy tudjuk, hol vagyunk
        # Az elcsúszó odometria helyett az AMCL abszolút, térkép-szintű pozícióját figyeljük!
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.pub_combat_status = self.create_publisher(Bool, '/combat_status', 10)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_shot_time = 0.0
        self.reload_time = 3.0  # 3 másodperc újratöltés
        
        # Az 5 ellenséges tank kezdő koordinátái az SDF alapján
        self.enemies = {
            1: (13.2575, 5.1433),
            2: (6.06511, 8.89161),
            3: (4.060240, 18.964700),
            4: (12.3271, 18.8093),
            5: (28.2249, 8.440780)
        }

        self.get_logger().info("💥 OKOS KILÖVŐ RENDSZER ONLINE! (Távolság alapú célpont választás)")

    def pose_callback(self, msg):
        # Folyamatosan frissítjük a pontos térkép-koordinátánkat
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def shoot_callback(self, msg):
        current_time = time.time()
        
        # Ha még töltünk, jelezzük a logban, és visszaengedjük a rendszert, hogy újra próbálkozhasson
        if current_time - self.last_shot_time < self.reload_time:
            remaining = self.reload_time - (current_time - self.last_shot_time)
            self.get_logger().warn( f"⏳ FEGYVER ÚJRATÖLTÉS ALATT! Még {remaining:.2f}s van hátra...")
            
            # Nem ragasztjuk be a státuszt, engedjük újraindulni a ciklust
            status_msg = Bool()
            status_msg.data = False
            self.pub_combat_status.publish(status_msg)
            return 
            
        self.last_shot_time = current_time

        self.get_logger().error("🎯 TŰZPARANCS ÉSZLELVE -> Legközelebbi ellenség elpárologtatása!")
        
        # Keresd meg a hozzánk legközelebbi tankot
        closest_id = None
        min_dist = float('inf')
        
        for tank_id, coords in self.enemies.items():
            dist = math.sqrt((self.robot_x - coords[0])**2 + (self.robot_y - coords[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_id = tank_id

        if closest_id is not None:
            # Gazebo remove parancs a KIVÁLASZTOTT tankra
            target_name = f"enemy_tank_{closest_id}"
            delete_cmd = f"gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req 'name: \"{target_name}\", type: MODEL'"
            os.system(delete_cmd)
            
            self.get_logger().fatal(f"💀 [{target_name}] sikeresen törölve a szimulációból! (Távolság: {min_dist:.2f}m)")
            
            # Töröljük a memóriából is, hogy a következőt keresse
            del self.enemies[closest_id]

        # Szólunk a turret_controllernek, hogy mehet tovább a navigáció
        status_msg = Bool()
        status_msg.data = False
        self.pub_combat_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShootControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()