import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
except Exception:
    cv2 = None

try:
    import depthai as dai
except Exception:
    dai = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


class OakNode(Node):
    def __init__(self) -> None:
        super().__init__('oak_node')

        self.rgb_pub = self.create_publisher(Image, '/voice_drone/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/voice_drone/depth', 10)

        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = float(self.declare_parameter('fps', 15.0).value)
        self.publish_depth = bool(self.declare_parameter('publish_depth', True).value)
        self.show = bool(self.declare_parameter('show', False).value)

        self.br = CvBridge() if CvBridge is not None else None
        self._stop = False
        self._th = None

        if dai is None or self.br is None:
            self.get_logger().error('depthai or cv_bridge is missing, oak node disabled')
            return

        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        self.get_logger().info('oak node ready')

    def _make_pipeline(self):
        p = dai.Pipeline()

        cam_rgb = p.create(dai.node.ColorCamera)
        xout_rgb = p.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')

        cam_rgb.setPreviewSize(self.width, self.height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(self.fps)
        cam_rgb.preview.link(xout_rgb.input)

        if self.publish_depth:
            mono_left = p.create(dai.node.MonoCamera)
            mono_right = p.create(dai.node.MonoCamera)
            stereo = p.create(dai.node.StereoDepth)
            xout_depth = p.create(dai.node.XLinkOut)
            xout_depth.setStreamName('depth')

            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setFps(self.fps)
            mono_right.setFps(self.fps)

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)
            stereo.depth.link(xout_depth.input)

        return p

    def _pub_rgb(self, frame) -> None:
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb'
        self.rgb_pub.publish(msg)

    def _pub_depth(self, frame) -> None:
        msg = self.br.cv2_to_imgmsg(frame, encoding='16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_depth'
        self.depth_pub.publish(msg)

    def _run(self) -> None:
        try:
            pipe = self._make_pipeline()
            with dai.Device(pipe) as dev:
                q_rgb = dev.getOutputQueue(name='rgb', maxSize=4, blocking=False)
                q_depth = None
                if self.publish_depth:
                    q_depth = dev.getOutputQueue(name='depth', maxSize=4, blocking=False)

                while rclpy.ok() and not self._stop:
                    got = False

                    in_rgb = q_rgb.tryGet()
                    if in_rgb is not None:
                        frame = in_rgb.getCvFrame()
                        self._pub_rgb(frame)
                        if self.show and cv2 is not None:
                            cv2.imshow('oak_rgb', frame)
                            cv2.waitKey(1)
                        got = True

                    if q_depth is not None:
                        in_depth = q_depth.tryGet()
                        if in_depth is not None:
                            depth = in_depth.getFrame()
                            self._pub_depth(depth)
                            got = True

                    if not got:
                        time.sleep(0.005)

        except Exception as e:
            self.get_logger().error(f'oak node failed: {e}')

    def destroy_node(self):
        self._stop = True
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
