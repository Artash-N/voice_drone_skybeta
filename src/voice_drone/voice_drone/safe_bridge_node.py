import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import jload


class SafeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('safe_bridge_node')

        self.mode = str(self.declare_parameter('mode', 'fake').value).strip().lower()

        self.sub = self.create_subscription(String, '/voice_drone/high_level_cmd', self._cb, 10)
        self.pub = self.create_publisher(String, '/voice_drone/safe_cmd', 10)
        self.last_warn_t = 0.0

        self.get_logger().info(f'safe bridge ready, mode={self.mode}')

    def _cb(self, msg: String) -> None:
        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return

        kind = str(obj.get('kind', ''))
        if kind in ['arm_takeoff', 'land', 'move_body', 'rotate', 'track', 'search', 'disarm', 'hold']:
            now = time.time()
            if now - self.last_warn_t > 1.0:
                self.last_warn_t = now
                self.get_logger().warn(f'live vehicle motion blocked in safe build, passing only to {self.mode}')

        if self.mode == 'fake':
            self.pub.publish(msg)
        else:
            # dry run only
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
