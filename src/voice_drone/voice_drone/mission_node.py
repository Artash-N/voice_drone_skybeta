import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import clamp, jdump, jload, label_match


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        self.intent_sub = self.create_subscription(String, '/voice_drone/intent', self._intent_cb, 10)
        self.det_sub = self.create_subscription(String, '/voice_drone/detections', self._det_cb, 10)

        self.cmd_pub = self.create_publisher(String, '/voice_drone/high_level_cmd', 10)
        self.state_pub = self.create_publisher(String, '/voice_drone/mission_state', 10)

        self.default_takeoff_alt_m = float(self.declare_parameter('default_takeoff_alt_m', 1.5).value)
        self.stop_dist_m = float(self.declare_parameter('stop_dist_m', 1.0).value)
        self.center_deadband = float(self.declare_parameter('center_deadband', 0.12).value)
        self.search_yaw_rate_dps = float(self.declare_parameter('search_yaw_rate_dps', 18.0).value)
        self.max_yaw_rate_dps = float(self.declare_parameter('max_yaw_rate_dps', 40.0).value)
        self.yaw_k = float(self.declare_parameter('yaw_k', 30.0).value)
        self.forward_k = float(self.declare_parameter('forward_k', 0.45).value)
        self.max_forward_mps = float(self.declare_parameter('max_forward_mps', 0.35).value)

        self.last_dets: List[Dict[str, Any]] = []
        self.last_det_t = 0.0

        self.state: Dict[str, Any] = {
            'mode': 'idle',
            'target': '',
            'tracking': False,
            'last_raw': '',
            'last_seen_label': '',
            'last_seen_depth_m': None,
            'last_action_t': 0.0,
        }

        self.last_cmd_log_t = 0.0

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('mission node ready')

    def _pub_cmd(self, obj: Dict[str, Any]) -> None:
        obj['t'] = time.time()
        obj['source'] = 'mission'
        msg = String()
        msg.data = jdump(obj)
        self.cmd_pub.publish(msg)
        self.state['last_action_t'] = obj['t']

        kind = str(obj.get('kind', ''))
        now = time.time()
        if kind not in ['search', 'track'] or (now - self.last_cmd_log_t > 0.75):
            self.last_cmd_log_t = now
            self.get_logger().info(f'cmd: {msg.data}')

    def _pub_state(self) -> None:
        obj = dict(self.state)
        obj['t'] = time.time()
        msg = String()
        msg.data = jdump(obj)
        self.state_pub.publish(msg)

    def _intent_cb(self, msg: String) -> None:
        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return

        action = str(obj.get('action', 'none'))
        raw = str(obj.get('raw', ''))
        self.state['last_raw'] = raw

        if action == 'status':
            self._pub_state()
            return

        if action == 'cancel':
            self.state['mode'] = 'idle'
            self.state['target'] = ''
            self.state['tracking'] = False
            self._pub_cmd({'kind': 'hold'})
            self._pub_state()
            return

        if action == 'disarm':
            self.state['mode'] = 'idle'
            self.state['target'] = ''
            self.state['tracking'] = False
            self._pub_cmd({'kind': 'disarm'})
            self._pub_state()
            return

        if action == 'hold':
            self.state['mode'] = 'hold'
            self.state['target'] = ''
            self.state['tracking'] = False
            self._pub_cmd({'kind': 'hold'})
            self._pub_state()
            return

        if action == 'land':
            self.state['mode'] = 'land'
            self.state['target'] = ''
            self.state['tracking'] = False
            self._pub_cmd({'kind': 'land'})
            self._pub_state()
            return

        if action == 'arm_takeoff':
            alt_m = float(obj.get('alt_m', self.default_takeoff_alt_m) or self.default_takeoff_alt_m)
            self.state['mode'] = 'takeoff'
            self.state['tracking'] = False
            self.state['target'] = ''
            self._pub_cmd({'kind': 'arm_takeoff', 'alt_m': alt_m})
            self._pub_state()
            return

        if action == 'move_body':
            mv = obj.get('move', {})
            self.state['mode'] = 'move_body'
            self.state['tracking'] = False
            self.state['target'] = ''
            self._pub_cmd({
                'kind': 'move_body',
                'x_m': float(mv.get('x_m', 0.0)),
                'y_m': float(mv.get('y_m', 0.0)),
                'z_m': float(mv.get('z_m', 0.0)),
            })
            self._pub_state()
            return

        if action == 'rotate':
            self.state['mode'] = 'rotate'
            self.state['tracking'] = False
            self.state['target'] = ''
            self._pub_cmd({
                'kind': 'rotate',
                'yaw_deg': float(obj.get('yaw_deg', 0.0)),
            })
            self._pub_state()
            return

        if action == 'go_to_object':
            tgt = str(obj.get('target', '')).strip()
            if not tgt:
                self.get_logger().warn('go_to_object came with no target')
                return
            self.state['mode'] = 'track'
            self.state['target'] = tgt
            self.state['tracking'] = True
            self._pub_state()
            return

        self._pub_state()

    def _det_cb(self, msg: String) -> None:
        obj = jload(msg.data, {})
        if not isinstance(obj, dict):
            return
        items = obj.get('items', [])
        if not isinstance(items, list):
            items = []
        self.last_dets = items
        self.last_det_t = time.time()

    def _best_match(self, target: str) -> Optional[Dict[str, Any]]:
        best = None
        best_score = -1.0
        for item in self.last_dets:
            if not isinstance(item, dict):
                continue
            lab = str(item.get('label', ''))
            if not label_match(lab, target):
                continue
            conf = float(item.get('conf', 0.0))
            area = 0.0
            bbox = item.get('bbox', [0, 0, 0, 0])
            if isinstance(bbox, list) and len(bbox) == 4:
                try:
                    area = max(0.0, float((bbox[2] - bbox[0]) * (bbox[3] - bbox[1])))
                except Exception:
                    area = 0.0
            score = conf + min(area / 200000.0, 1.0)
            if score > best_score:
                best = item
                best_score = score
        return best

    def _tick(self) -> None:
        if not self.state.get('tracking', False):
            self._pub_state()
            return

        target = str(self.state.get('target', ''))
        if not target:
            self.state['tracking'] = False
            self.state['mode'] = 'idle'
            self._pub_state()
            return

        item = self._best_match(target)
        if item is None:
            self.state['last_seen_label'] = ''
            self.state['last_seen_depth_m'] = None
            self._pub_cmd({
                'kind': 'search',
                'target': target,
                'yaw_rate_dps': self.search_yaw_rate_dps,
            })
            self._pub_state()
            return

        cx = int(item.get('cx', 0))
        img_w = int(item.get('img_w', 1))
        ex = 0.0
        if img_w > 0:
            ex = (float(cx) - (float(img_w) * 0.5)) / (float(img_w) * 0.5)

        yaw_rate_dps = clamp(ex * self.yaw_k, -self.max_yaw_rate_dps, self.max_yaw_rate_dps)
        if abs(ex) < self.center_deadband:
            yaw_rate_dps = 0.0

        dist_m = item.get('depth_m', None)
        if dist_m is not None:
            try:
                dist_m = float(dist_m)
            except Exception:
                dist_m = None

        fwd = 0.0
        if dist_m is not None and abs(ex) <= self.center_deadband and dist_m > self.stop_dist_m:
            fwd = clamp((dist_m - self.stop_dist_m) * self.forward_k, 0.0, self.max_forward_mps)

        self.state['last_seen_label'] = str(item.get('label', ''))
        self.state['last_seen_depth_m'] = dist_m

        if dist_m is not None and dist_m <= self.stop_dist_m and abs(ex) <= self.center_deadband:
            self._pub_cmd({'kind': 'hold'})
            self.state['mode'] = 'hold'
            self.state['tracking'] = False
            self._pub_state()
            return

        self._pub_cmd({
            'kind': 'track',
            'target': target,
            'label': str(item.get('label', '')),
            'depth_m': dist_m,
            'yaw_rate_dps': yaw_rate_dps,
            'forward_mps': fwd,
        })
        self._pub_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
