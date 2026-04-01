import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .common import get_env, jdump

try:
    from px4_msgs.msg import VehicleLocalPosition
    from px4_msgs.msg import VehicleStatus
    from px4_msgs.msg import DistanceSensor
    from px4_msgs.msg import SensorOpticalFlow
except Exception:
    VehicleLocalPosition = None
    VehicleStatus = None
    DistanceSensor = None
    SensorOpticalFlow = None


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_node')

        self.pub = self.create_publisher(String, '/voice_drone/telemetry', 10)

        px4_ns_param = str(self.declare_parameter('px4_ns', '').value)
        env_ns = get_env('PX4_TOPIC_NS', '')
        self.px4_ns = (px4_ns_param or env_ns).strip()
        self.publish_hz = float(self.declare_parameter('publish_hz', 5.0).value)

        self.state: Dict[str, Any] = {
            'px4_ok': False,
            'armed': None,
            'nav_state': None,
            'x': None,
            'y': None,
            'z': None,
            'vx': None,
            'vy': None,
            'vz': None,
            'heading': None,
            'range_m': None,
            'of_quality': None,
            'of_dist_m': None,
            'last_seen_t': None,
        }

        if VehicleLocalPosition is None or VehicleStatus is None:
            self.get_logger().error('px4_msgs is missing, telemetry node disabled')
            return

        self._mk_subs()
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 0.5), self._tick)
        self.get_logger().info(f'telemetry node ready, ns="{self.px4_ns}"')

    def _topic(self, name: str) -> str:
        ns = self.px4_ns.strip('/')
        if not ns:
            return f'/fmu/out/{name}'
        return f'/{ns}/fmu/out/{name}'

    def _mk_subs(self) -> None:
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition,
            self._topic('vehicle_local_position'),
            self._pos_cb,
            qos_profile_sensor_data,
        )
        self.sub_status = self.create_subscription(
            VehicleStatus,
            self._topic('vehicle_status'),
            self._status_cb,
            qos_profile_sensor_data,
        )

        self.sub_rng = None
        self.sub_of = None

        if DistanceSensor is not None:
            self.sub_rng = self.create_subscription(
                DistanceSensor,
                self._topic('distance_sensor'),
                self._rng_cb,
                qos_profile_sensor_data,
            )

        if SensorOpticalFlow is not None:
            self.sub_of = self.create_subscription(
                SensorOpticalFlow,
                self._topic('sensor_optical_flow'),
                self._of_cb,
                qos_profile_sensor_data,
            )

    def _mark_seen(self) -> None:
        self.state['px4_ok'] = True
        self.state['last_seen_t'] = time.time()

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.state['x'] = float(msg.x)
        self.state['y'] = float(msg.y)
        self.state['z'] = float(msg.z)
        self.state['vx'] = float(msg.vx)
        self.state['vy'] = float(msg.vy)
        self.state['vz'] = float(msg.vz)
        self.state['heading'] = float(msg.heading)
        self._mark_seen()

    def _status_cb(self, msg: VehicleStatus) -> None:
        armed = None
        try:
            armed = (int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED))
        except Exception:
            armed = None
        self.state['armed'] = armed
        self.state['nav_state'] = int(msg.nav_state)
        self._mark_seen()

    def _rng_cb(self, msg: DistanceSensor) -> None:
        try:
            self.state['range_m'] = float(msg.current_distance)
        except Exception:
            self.state['range_m'] = None
        self._mark_seen()

    def _of_cb(self, msg: SensorOpticalFlow) -> None:
        try:
            self.state['of_quality'] = int(msg.quality)
        except Exception:
            self.state['of_quality'] = None
        try:
            self.state['of_dist_m'] = float(msg.distance_m)
        except Exception:
            self.state['of_dist_m'] = None
        self._mark_seen()

    def _tick(self) -> None:
        msg = String()
        out = dict(self.state)
        out['t'] = time.time()
        msg.data = jdump(out)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
