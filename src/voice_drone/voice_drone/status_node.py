import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import jload


class StatusNode(Node):
    def __init__(self) -> None:
        super().__init__('status_node')

        self.det_print_hz = float(self.declare_parameter('det_print_hz', 1.0).value)
        self.state_print_hz = float(self.declare_parameter('state_print_hz', 2.0).value)

        self.create_subscription(String, '/voice_drone/raw_text', self._raw_cb, 10)
        self.create_subscription(String, '/voice_drone/intent', self._intent_cb, 10)
        self.create_subscription(String, '/voice_drone/detections', self._det_cb, 10)
        self.create_subscription(String, '/voice_drone/mission_state', self._mission_cb, 10)
        self.create_subscription(String, '/voice_drone/telemetry', self._telemetry_cb, 10)
        self.create_subscription(String, '/voice_drone/fake_state', self._fake_cb, 10)

        self.last_det_print = 0.0
        self.last_state_print = 0.0
        self.last_fake_print = 0.0
        self.last_px4_print = 0.0

        self.get_logger().info('status node ready')

    def _raw_cb(self, msg: String) -> None:
        print(f'[heard] {msg.data}')

    def _intent_cb(self, msg: String) -> None:
        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        act = obj.get('action', 'none')
        tgt = obj.get('target', '')
        print(f'[intent] action={act} target={tgt}')

    def _det_cb(self, msg: String) -> None:
        now = time.time()
        if now - self.last_det_print < (1.0 / max(self.det_print_hz, 0.1)):
            return
        self.last_det_print = now

        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        items = obj.get('items', [])
        if not items:
            return
        top = items[0]
        lab = top.get('label', '')
        conf = top.get('conf', 0.0)
        dep = top.get('depth_m', None)
        if dep is None:
            print(f'[det] {lab} conf={conf}')
        else:
            print(f'[det] {lab} conf={conf} depth={dep}m')

    def _mission_cb(self, msg: String) -> None:
        now = time.time()
        if now - self.last_state_print < (1.0 / max(self.state_print_hz, 0.1)):
            return
        self.last_state_print = now

        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        print(f'[mission] mode={obj.get("mode")} target={obj.get("target")} tracking={obj.get("tracking")}')

    def _telemetry_cb(self, msg: String) -> None:
        now = time.time()
        if now - self.last_px4_print < (1.0 / max(self.state_print_hz, 0.1)):
            return
        self.last_px4_print = now

        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        if not obj.get('px4_ok', False):
            return
        z = obj.get('z', None)
        armed = obj.get('armed', None)
        rng = obj.get('range_m', None)
        if rng is None:
            print(f'[px4] armed={armed} z={z}')
        else:
            print(f'[px4] armed={armed} z={z} range={rng}')

    def _fake_cb(self, msg: String) -> None:
        now = time.time()
        if now - self.last_fake_print < (1.0 / max(self.state_print_hz, 0.1)):
            return
        self.last_fake_print = now

        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        x = float(obj.get('x_m', 0.0))
        y = float(obj.get('y_m', 0.0))
        z = float(obj.get('z_m', 0.0))
        yaw = float(obj.get('yaw_deg', 0.0))
        print(f'[fake] mode={obj.get("mode")} pos=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.1f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
