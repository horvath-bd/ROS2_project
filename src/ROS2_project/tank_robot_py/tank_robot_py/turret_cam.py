import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os

class ImageGrabberNode(Node):
    def __init__(self):
        super().__init__('image_grabber_node')
        self.br = CvBridge()
        
        # Feliratkozás a tankod tömörített kameraképére
        self.sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        
        # Mappa létrehozása a képeknek
        self.output_dir = 'tank_kepek'
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.img_count = 0
        self.get_logger().info(f"📸 Képgyűjtő elindult! Mentési mappa: ./{self.output_dir}")
        self.get_logger().info("Nyomj [SPACE]-t a mentéshez, [ESC]-et a kilépéshez!")

    def image_callback(self, msg):
        # ROS kép átalakítása OpenCV képpé (Ez a tiszta, eredeti 640x480-as kép)
        cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --- KÉPMÉRET MEGDUPLÁZÁSA CSAK A MEGJELENÍTÉSHEZ ---
        # cv2.INTER_CUBIC: intelligens simítás, hogy ne legyen pixeles a nagyítás!
        height, width = cv_image.shape[:2]
        large_image = cv2.resize(cv_image, (width * 2, height * 2), interpolation=cv2.INTER_CUBIC)
        
        # Opcionális: Dobunk rá egy kis infót, hogy lásd a képernyőn is a mentési állapotot
        cv2.putText(large_image, f"Total Saved: {self.img_count}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # A NAGYÍTOTT képet mutatjuk neked az ablakban
        cv2.imshow("TANK KAMERA - ADATGYUJTAS", large_image)
        key = cv2.waitKey(1) & 0xFF
        
        # Ha szóközt nyomsz (ASCII 32)
        if key == 32:
            self.img_count += 1
            filename = os.path.join(self.output_dir, f"tank_frame_{self.img_count:03d}.jpg")
            
            # --- FONTOS: Az EREDETI cv_image-t mentjük, nem a felnagyítottat! ---
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"💾 Eredeti méretű (640x480) kép elmentve: {filename}")
            
        # Ha ESC-et nyomsz (ASCII 27)
        elif key == 27:
            self.get_logger().info("Kilépés...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageGrabberNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

