import copy
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import (
    INTENT_SCHEMA,
    call_openai_json,
    canon_target,
    clean_text,
    fallback_parse_intent,
    get_env,
    jdump,
)


SYS_PROMPT = """
Turn one short drone command into exactly one JSON object.

Allowed actions:
- none
- arm_takeoff
- land
- hold
- go_to_object
- move_body
- rotate
- disarm
- cancel
- status

Rules:
- "stop", "hover", and "hold position" mean hold.
- "arm and takeoff" means arm_takeoff.
- "go to coke can" means go_to_object with target "coke can".
- If altitude is missing for arm_takeoff, use the default altitude already implied by the app.
- For move_body use x_m forward, y_m left, z_m up.
- For rotate use yaw_deg where right/clockwise is positive and left/counterclockwise is negative.
- If the command does not fit, action should be "none".
- Keep raw equal to the original command text.
- Output only the JSON that matches the schema.
""".strip()


class IntentNode(Node):
    def __init__(self) -> None:
        super().__init__('intent_node')

        self.sub = self.create_subscription(String, '/voice_drone/raw_text', self._raw_cb, 10)
        self.pub = self.create_publisher(String, '/voice_drone/intent', 10)

        self.api_key = get_env('OPENAI_API_KEY', '')
        self.base_url = get_env('OPENAI_BASE_URL', 'https://api.openai.com/v1')
        self.use_openai = bool(self.declare_parameter('use_openai', True).value)
        self.openai_timeout_sec = float(self.declare_parameter('openai_timeout_sec', 40.0).value)
        intent_model_default = get_env('OPENAI_INTENT_MODEL', 'gpt-4.1')
        self.intent_model = str(self.declare_parameter('intent_model', intent_model_default).value)
        self.default_takeoff_alt_m = float(self.declare_parameter('default_takeoff_alt_m', 1.5).value)

        self.q: "queue.Queue[str]" = queue.Queue()
        self._stop = False
        self.t = threading.Thread(target=self._worker, daemon=True)
        self.t.start()

        self.get_logger().info('intent node ready')

    def _raw_cb(self, msg: String) -> None:
        txt = clean_text(msg.data)
        if txt:
            self.q.put(txt)

    def _do_openai(self, txt: str):
        if not self.use_openai:
            return None
        if not self.api_key:
            return None

        try:
            obj = call_openai_json(
                raw_text=txt,
                api_key=self.api_key,
                model=self.intent_model,
                base_url=self.base_url,
                schema=INTENT_SCHEMA,
                instructions=SYS_PROMPT,
                timeout_sec=self.openai_timeout_sec,
            )
            return obj
        except Exception as e:
            self.get_logger().warn(f'openai parse failed, using fallback: {e}')
            return None

    def _fix_obj(self, obj, txt: str):
        if not isinstance(obj, dict):
            obj = fallback_parse_intent(txt, self.default_takeoff_alt_m)

        out = copy.deepcopy(obj)

        out.setdefault('ok', True)
        out.setdefault('action', 'none')
        out.setdefault('target', '')
        out.setdefault('alt_m', 0.0)
        out.setdefault('dist_m', 0.0)
        out.setdefault('yaw_deg', 0.0)
        out.setdefault('move', {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0})
        out.setdefault('raw', txt)

        if not isinstance(out.get('move'), dict):
            out['move'] = {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0}

        out['move'].setdefault('x_m', 0.0)
        out['move'].setdefault('y_m', 0.0)
        out['move'].setdefault('z_m', 0.0)

        out['target'] = canon_target(str(out.get('target', '')).strip())
        out['raw'] = txt

        return out

    def _worker(self) -> None:
        while not self._stop and rclpy.ok():
            try:
                txt = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                obj = self._do_openai(txt)
                if obj is None:
                    obj = fallback_parse_intent(txt, self.default_takeoff_alt_m)
                obj = self._fix_obj(obj, txt)

                msg = String()
                msg.data = jdump(obj)
                self.pub.publish(msg)
                self.get_logger().info(f'intent: {msg.data}')
            except Exception as e:
                self.get_logger().error(f'intent worker failed: {e}')

    def destroy_node(self):
        self._stop = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
