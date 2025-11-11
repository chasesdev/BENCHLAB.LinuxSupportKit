# ROS 2 node that subscribes to the BenchLab service NDJSON stream and publishes a sample std_msgs/String
# Requires: rclpy, requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class BenchLabNode(Node):
    def __init__(self):
        super().__init__('benchlab_ros2')
        self.pub = self.create_publisher(String, '/benchlab/raw_hex', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.session = requests.Session()
        self.stream_iter = None

    def tick(self):
        try:
            if self.stream_iter is None:
                r = self.session.get('http://127.0.0.1:8080/stream', stream=True, timeout=5)
                r.raise_for_status()
                self.stream_iter = r.iter_lines()
            for _ in range(10):
                line = next(self.stream_iter)
                if not line:
                    continue
                msg = String()
                msg.data = line.decode()
                self.pub.publish(msg)
        except StopIteration:
            self.stream_iter = None
        except Exception as e:
            self.get_logger().warn(f"stream error: {e}")
            self.stream_iter = None

def main():
    rclpy.init()
    node = BenchLabNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
