import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
import os
import time
import math

class ShootControllerNode(Node):
    def __init__(self):
        super().__init__('shoot_controller_node')
        
        self.sub_shoot = self.create_subscription(Empty, '/shoot', self.shoot_callback, 10)
        # Feliratkozunk az odometriára, hogy tudjuk, hol vagyunk
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.pub_combat_status = self.create_publisher(Bool, '/combat_status', 10)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_shot_time = 0.0
        self.reload_time = 3.0  # 3 másodperc újratöltés
        
        # Az 5 ellenséges tank kezdő koordinátái az SDF alapján
        self.enemies = {
            1: (13.0, 0.0),
            2: (13.0, 13.0),
            3: (1.0, 15.0),
            4: (18.0, 18.0),
            5: (18.0, 0.0)
        }

        self.get_logger().info("💥 OKOS KILÖVŐ RENDSZER ONLINE! (Távolság alapú célpont választás)")

    def odom_callback(self, msg):
        # Folyamatosan frissítjük a saját pozíciónkat
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def shoot_callback(self, msg):
        current_time = time.time()
        # Ha még töltünk, ignoráljuk a lövést
        if current_time - self.last_shot_time < self.reload_time:
            return 
            
        self.last_shot_time = current_time

        if not self.enemies:
            self.get_logger().info("🏆 Minden ellenség megsemmisítve!")
            status_msg = Bool()
            status_msg.data = False
            self.pub_combat_status.publish(status_msg)
            return

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