import math
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import clamp, jdump, jload


def wrap_deg(x: float) -> float:
    while x > 180.0:
        x -= 360.0
    while x < -180.0:
        x += 360.0
    return x


class FakeDroneNode(Node):
    def __init__(self) -> None:
        super().__init__('fake_drone_node')

        self.sub = self.create_subscription(String, '/voice_drone/safe_cmd', self._cb, 10)
        self.pub = self.create_publisher(String, '/voice_drone/fake_state', 10)

        self.climb_mps = float(self.declare_parameter('climb_mps', 0.6).value)
        self.move_mps = float(self.declare_parameter('move_mps', 0.5).value)
        self.rotate_dps = float(self.declare_parameter('rotate_dps', 45.0).value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 10.0).value)

        self.state: Dict[str, Any] = {
            'armed': False,
            'air': False,
            'mode': 'idle',
            'x_m': 0.0,
            'y_m': 0.0,
            'z_m': 0.0,
            'yaw_deg': 0.0,
            't': time.time(),
        }

        self.target_alt: Optional[float] = None
        self.move_target = None
        self.yaw_target = None

        self.cmd_until = 0.0
        self.track_fwd = 0.0
        self.track_yaw_rate = 0.0

        self.last_t = time.time()
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1.0), self._tick)

        self.get_logger().info('fake drone ready')

    def _cb(self, msg: String) -> None:
        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return

        kind = str(obj.get('kind', ''))
        now = time.time()

        if kind == 'arm_takeoff':
            self.state['armed'] = True
            self.state['air'] = True
            self.state['mode'] = 'takeoff'
            self.target_alt = max(0.3, float(obj.get('alt_m', 1.5)))
            self.move_target = None
            self.yaw_target = None

        elif kind == 'land':
            self.state['mode'] = 'land'
            self.target_alt = 0.0
            self.move_target = None

        elif kind == 'hold':
            self.state['mode'] = 'hold'
            self.move_target = None
            self.yaw_target = None
            self.track_fwd = 0.0
            self.track_yaw_rate = 0.0
            self.cmd_until = 0.0

        elif kind == 'disarm':
            self.state['armed'] = False
            self.state['air'] = False
            self.state['mode'] = 'idle'
            self.state['z_m'] = 0.0
            self.target_alt = None
            self.move_target = None
            self.yaw_target = None
            self.track_fwd = 0.0
            self.track_yaw_rate = 0.0
            self.cmd_until = 0.0

        elif kind == 'rotate':
            self.state['mode'] = 'rotate'
            self.yaw_target = wrap_deg(self.state['yaw_deg'] + float(obj.get('yaw_deg', 0.0)))

        elif kind == 'move_body':
            self.state['mode'] = 'move_body'
            x_m = float(obj.get('x_m', 0.0))
            y_m = float(obj.get('y_m', 0.0))
            z_m = float(obj.get('z_m', 0.0))

            yaw = math.radians(float(self.state['yaw_deg']))
            dx = x_m * math.cos(yaw) - y_m * math.sin(yaw)
            dy = x_m * math.sin(yaw) + y_m * math.cos(yaw)
            dz = z_m

            self.move_target = {
                'x_m': self.state['x_m'] + dx,
                'y_m': self.state['y_m'] + dy,
                'z_m': max(0.0, self.state['z_m'] + dz),
            }

        elif kind == 'search':
            self.state['mode'] = 'search'
            self.track_fwd = 0.0
            self.track_yaw_rate = float(obj.get('yaw_rate_dps', 0.0))
            self.cmd_until = now + 0.5

        elif kind == 'track':
            self.state['mode'] = 'track'
            self.track_fwd = float(obj.get('forward_mps', 0.0))
            self.track_yaw_rate = float(obj.get('yaw_rate_dps', 0.0))
            self.cmd_until = now + 0.5

    def _step_to(self, cur: float, tgt: float, rate: float, dt: float) -> float:
        if cur < tgt:
            return min(cur + rate * dt, tgt)
        return max(cur - rate * dt, tgt)

    def _tick(self) -> None:
        now = time.time()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now

        mode = str(self.state['mode'])

        if mode == 'takeoff' and self.target_alt is not None:
            self.state['z_m'] = self._step_to(float(self.state['z_m']), float(self.target_alt), self.climb_mps, dt)
            if abs(float(self.state['z_m']) - float(self.target_alt)) < 0.03:
                self.state['mode'] = 'hold'

        elif mode == 'land':
            self.state['z_m'] = max(0.0, float(self.state['z_m']) - self.climb_mps * dt)
            if float(self.state['z_m']) <= 0.01:
                self.state['z_m'] = 0.0
                self.state['air'] = False
                self.state['armed'] = False
                self.state['mode'] = 'idle'

        elif mode == 'rotate' and self.yaw_target is not None:
            cur = float(self.state['yaw_deg'])
            tgt = float(self.yaw_target)
            diff = wrap_deg(tgt - cur)
            step = clamp(diff, -self.rotate_dps * dt, self.rotate_dps * dt)
            self.state['yaw_deg'] = wrap_deg(cur + step)
            if abs(wrap_deg(float(self.yaw_target) - float(self.state['yaw_deg']))) < 1.0:
                self.state['mode'] = 'hold'
                self.yaw_target = None

        elif mode == 'move_body' and self.move_target is not None:
            cur_x = float(self.state['x_m'])
            cur_y = float(self.state['y_m'])
            cur_z = float(self.state['z_m'])
            tgt_x = float(self.move_target['x_m'])
            tgt_y = float(self.move_target['y_m'])
            tgt_z = float(self.move_target['z_m'])

            dx = tgt_x - cur_x
            dy = tgt_y - cur_y
            dz = tgt_z - cur_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < 0.03:
                self.state['x_m'] = tgt_x
                self.state['y_m'] = tgt_y
                self.state['z_m'] = tgt_z
                self.state['mode'] = 'hold'
                self.move_target = None
            else:
                step = min(self.move_mps * dt, dist)
                self.state['x_m'] = cur_x + (dx / dist) * step
                self.state['y_m'] = cur_y + (dy / dist) * step
                self.state['z_m'] = cur_z + (dz / dist) * step

        elif mode in ['track', 'search']:
            if now > self.cmd_until:
                self.track_fwd = 0.0
                self.track_yaw_rate = 0.0
                self.state['mode'] = 'hold'
            else:
                self.state['yaw_deg'] = wrap_deg(float(self.state['yaw_deg']) + self.track_yaw_rate * dt)
                yaw = math.radians(float(self.state['yaw_deg']))
                self.state['x_m'] = float(self.state['x_m']) + math.cos(yaw) * self.track_fwd * dt
                self.state['y_m'] = float(self.state['y_m']) + math.sin(yaw) * self.track_fwd * dt

        self.state['air'] = bool(float(self.state['z_m']) > 0.05)
        self.state['t'] = now

        msg = String()
        msg.data = jdump(self.state)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
