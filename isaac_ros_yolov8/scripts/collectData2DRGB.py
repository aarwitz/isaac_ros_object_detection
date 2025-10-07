import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.save_dir = os.path.join(script_dir, 'captured_images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Find the highest existing image number to continue from there
        self.counter = self.find_next_counter()
        self.get_logger().info(f'Starting from image_{self.counter:04d}.jpg')
        self.last_save_time = 0  # Initialize to 0 to save first image immediately

    def find_next_counter(self):
        """Find the highest numbered image file and return the next number to use."""
        if not os.path.exists(self.save_dir):
            return 0
        
        max_num = -1
        for filename in os.listdir(self.save_dir):
            if filename.startswith('image_') and filename.endswith('.jpg'):
                try:
                    # Extract number from filename like 'image_0063.jpg'
                    num_str = filename[6:10]  # Get the 4-digit number part
                    num = int(num_str)
                    max_num = max(max_num, num)
                except (ValueError, IndexError):
                    continue
        
        return max_num + 1 if max_num >= 0 else 0

    def listener_callback(self, msg):
        import time
        current_time = time.time()

        # Save image every 1 second
        if current_time - self.last_save_time >= 1.0:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.save_dir, f'image_{self.counter:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved: {filename}')
            self.counter += 1
            self.last_save_time = current_time
            
            if self.counter >= 100:  # stop after 100 images
                self.get_logger().info('Captured 100 images. Shutting down.')
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
