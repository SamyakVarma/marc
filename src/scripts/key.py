import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from key_data import KeyboardCoordinates
import time

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.key_pub = self.create_publisher(Point, '/key_xyz', 10)
        self.keyboard = KeyboardCoordinates()
        self.word = "ASTRA"
        self.current_index = 0
        self.timer = self.create_timer(2.0, self.publish_coordinates)

    def publish_coordinates(self):
        if self.current_index < len(self.word):
            key = self.word[self.current_index]
            coordinates = self.keyboard.get_coordinates(key)
            if isinstance(coordinates, tuple):
                msg = Point()
                msg.x = coordinates[0]
                msg.y = coordinates[1]
                msg.z = 0.0
                self.key_pub.publish(msg)
                self.get_logger().info(f'Published coordinates for key "{key}": ({msg.x}, {msg.y})')
                self.current_index += 1
            else:
                self.get_logger().warning(coordinates)
        else:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    key_publisher = KeyPublisher()
    rclpy.spin(key_publisher)
    key_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
