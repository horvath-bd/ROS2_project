import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import subprocess
import threading

class HitDetectorNode(Node):
    def __init__(self):
        super().__init__('hit_detector_node')
        
        self.sub_shot = self.create_subscription(
            Float64MultiArray, 
            '/shot_info', 
            self.shot_callback, 
            10
        )
        
        # --- CÉLPONT POZÍCIÓJA (A tank KÖZEPE) ---
        self.enemy_x = 8.0
        self.enemy_y = 0.0
        self.enemy_z = 0.5
        
        # --- CÉLPONT PONTOS MÉRETEI ---
        self.target_size_x = 0.879  # Hosszúság
        self.target_size_y = 0.440  # Szélesség
        self.target_size_z = 0.276  # Magasság
        
        # --- FIZIKAI PARAMÉTEREK ---
        self.v0 = 20.0       # Lövedék kezdősebessége (m/s)
        self.g = 9.81        # Gravitáció (m/s^2)
        self.dt = 0.01       # Szimulációs időlépés
        
        self.enemy_destroyed = False
        self.get_logger().info("🚀 Ballisztikus Hitscan (AABB Hitbox) Élesítve!")

    def shot_callback(self, msg):
        if self.enemy_destroyed:
            return

        bullet_id, bx, by, bz, pitch, yaw = msg.data
        bullet_name = f"dynamic_bullet_{int(bullet_id)}"

        # 1. Sebességvektor komponenseinek kiszámítása
        v_x = self.v0 * math.cos(pitch) * math.cos(yaw)
        v_y = self.v0 * math.cos(pitch) * math.sin(yaw)
        v_z = self.v0 * math.sin(pitch)

        hit_detected = False
        t = 0.0

        # 2. Lövedék röppályájának szimulálása
        while True:
            # Jelenlegi pozíció a 't' időpillanatban
            current_x = bx + v_x * t
            current_y = by + v_y * t
            current_z = bz + v_z * t - 0.5 * self.g * (t ** 2)

            # Ha a lövedék a föld alá esik (z < 0), vége a röppályának
            if current_z < 0.0:
                self.get_logger().info("💨 Mellé: A lövedék a földbe csapódott.")
                break

            # Távolság a célpont közepétől
            dist = math.sqrt(
                (current_x - self.enemy_x)**2 + 
                (current_y - self.enemy_y)**2 + 
                (current_z - self.enemy_z)**2
            )

            # Találat ellenőrzése
            if dist <= self.hitbox_radius:
                hit_detected = True
                break

            # Idő léptetése
            t += self.dt

        # 3. Eredmény kiértékelése
        if hit_detected:
            self.get_logger().info(f"💥 BUMM! BALLISZTIKUS TALÁLAT! Repülési idő: {t:.2f}s")
            self.enemy_destroyed = True
            threading.Thread(target=self.remove_entities, args=(bullet_name,)).start()

    def remove_entities(self, bullet_name):
        import time
        # Várunk picit, hogy a látvány kedvéért a lövedék tényleg beérjen a Gazebóban is
        time.sleep(0.1) 
        
        self.get_logger().info("🧹 Célpont törlése...")
        
        remove_enemy_cmd = (
            f"gz service -s /world/empty/remove "
            f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 "
            f"--req \"name: 'enemy_tank', type: MODEL\""
        )
        subprocess.run(remove_enemy_cmd, shell=True, stdout=subprocess.DEVNULL)
        
        remove_bullet_cmd = (
            f"gz service -s /world/empty/remove "
            f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 "
            f"--req \"name: '{bullet_name}', type: MODEL\""
        )
        subprocess.run(remove_bullet_cmd, shell=True, stdout=subprocess.DEVNULL)
        
        self.get_logger().info("✅ Célpont megsemmisítve!")

def main(args=None):
    rclpy.init(args=args)
    node = HitDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()