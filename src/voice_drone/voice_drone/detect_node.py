import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import jdump
from .common import get_env

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class DetectNode(Node):
    def __init__(self) -> None:
        super().__init__('detect_node')

        model_path_default = get_env('YOLO_MODEL', 'yolov8n.pt')
        self.model_path = str(self.declare_parameter('model_path', model_path_default).value)
        self.conf = float(self.declare_parameter('conf', 0.35).value)
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        self.detect_hz = float(self.declare_parameter('detect_hz', 5.0).value)
        self.show = bool(self.declare_parameter('show', False).value)

        self.pub = self.create_publisher(String, '/voice_drone/detections', 10)
        self.rgb_sub = self.create_subscription(Image, '/voice_drone/rgb', self._rgb_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/voice_drone/depth', self._depth_cb, 10)

        self.br = CvBridge() if CvBridge is not None else None
        self.model = None
        self.names = {}

        self.depth_lock = threading.Lock()
        self.last_depth = None
        self.last_depth_shape = None

        self.busy = False
        self.last_run = 0.0

        if self.br is None or YOLO is None or np is None:
            self.get_logger().error('cv_bridge, ultralytics, or numpy is missing, detect node disabled')
            return

        try:
            self.model = YOLO(self.model_path)
            self.names = self.model.names
            self.get_logger().info(f'loaded yolo model: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'could not load model: {e}')

    def _depth_cb(self, msg: Image) -> None:
        if self.br is None:
            return
        try:
            depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.last_depth = depth.copy()
                self.last_depth_shape = depth.shape[:2]
        except Exception as e:
            self.get_logger().warn(f'depth conv failed: {e}')

    def _rgb_cb(self, msg: Image) -> None:
        if self.model is None or self.br is None:
            return

        now = time.time()
        if self.busy:
            return
        if now - self.last_run < (1.0 / max(self.detect_hz, 0.1)):
            return

        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'rgb conv failed: {e}')
            return

        self.last_run = now
        self.busy = True
        th = threading.Thread(target=self._run_det, args=(frame,), daemon=True)
        th.start()

    def _pick_depth(self, cx: int, cy: int, rgb_w: int, rgb_h: int) -> Optional[float]:
        with self.depth_lock:
            if self.last_depth is None:
                return None
            depth = self.last_depth.copy()

        if depth is None or np is None:
            return None

        dh, dw = depth.shape[:2]
        dx = int(cx * float(dw) / max(1.0, float(rgb_w)))
        dy = int(cy * float(dh) / max(1.0, float(rgb_h)))

        win = 4
        x1 = max(0, dx - win)
        x2 = min(dw, dx + win + 1)
        y1 = max(0, dy - win)
        y2 = min(dh, dy + win + 1)

        patch = depth[y1:y2, x1:x2]
        if patch.size == 0:
            return None

        vals = patch.reshape(-1)
        vals = vals[vals > 0]
        if vals.size == 0:
            return None

        med_mm = float(np.median(vals))
        return med_mm / 1000.0

    def _run_det(self, frame) -> None:
        try:
            res = self.model(frame, verbose=False, conf=self.conf, imgsz=self.imgsz)
            if not res:
                out = {'t': time.time(), 'items': []}
                msg = String()
                msg.data = jdump(out)
                self.pub.publish(msg)
                return

            r0 = res[0]
            items: List[Dict[str, Any]] = []
            rgb_h, rgb_w = frame.shape[:2]

            boxes = getattr(r0, 'boxes', None)
            if boxes is not None:
                for b in boxes:
                    try:
                        cls_id = int(b.cls[0].item())
                        conf = float(b.conf[0].item())
                        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                    except Exception:
                        continue

                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    lab = str(self.names.get(cls_id, str(cls_id)))
                    dist_m = self._pick_depth(cx, cy, rgb_w, rgb_h)

                    items.append({
                        'label': lab.lower(),
                        'conf': round(conf, 4),
                        'bbox': [x1, y1, x2, y2],
                        'cx': cx,
                        'cy': cy,
                        'img_w': rgb_w,
                        'img_h': rgb_h,
                        'depth_m': None if dist_m is None else round(dist_m, 3),
                    })

                    if self.show and cv2 is not None:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        txt = f'{lab} {conf:.2f}'
                        if dist_m is not None:
                            txt += f' {dist_m:.2f}m'
                        cv2.putText(frame, txt, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            out = {'t': time.time(), 'items': items}
            msg = String()
            msg.data = jdump(out)
            self.pub.publish(msg)

            if self.show and cv2 is not None:
                cv2.imshow('voice_drone_dets', frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'detect failed: {e}')
        finally:
            self.busy = False

    def destroy_node(self):
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
