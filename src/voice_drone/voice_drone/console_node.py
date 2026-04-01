import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import call_openai_transcribe_file, clean_text, get_env, save_wav_int16, tmp_wav_path

try:
    import numpy as np
except Exception:
    np = None

try:
    import sounddevice as sd
except Exception:
    sd = None


class ConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('console_node')

        self.pub = self.create_publisher(String, '/voice_drone/raw_text', 10)

        self.rec_secs = float(self.declare_parameter('rec_secs', 4.0).value)
        self.sample_rate = int(self.declare_parameter('sample_rate', 16000).value)
        self.channels = int(self.declare_parameter('channels', 1).value)
        self.transcribe_timeout_sec = float(self.declare_parameter('transcribe_timeout_sec', 60.0).value)
        self.api_key = get_env('OPENAI_API_KEY', '')
        self.base_url = get_env('OPENAI_BASE_URL', 'https://api.openai.com/v1')
        transcribe_model_default = get_env('OPENAI_TRANSCRIBE_MODEL', 'gpt-4o-mini-transcribe')
        self.transcribe_model = str(self.declare_parameter('transcribe_model', transcribe_model_default).value)

        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self._print_help()
        self.get_logger().info('console node ready')

    def _print_help(self) -> None:
        print('')
        print('type plain text commands, or use:')
        print('  /rec 4    record 4 seconds from mic and transcribe')
        print('  /help     print help')
        print('  /quit     stop ros2')
        print('')

    def _publish_text(self, txt: str) -> None:
        msg = String()
        msg.data = txt
        self.pub.publish(msg)
        self.get_logger().info(f'heard: {txt}')

    def _record_and_transcribe(self, secs: float) -> None:
        if sd is None or np is None:
            self.get_logger().error('sounddevice or numpy is not installed')
            return
        if not self.api_key:
            self.get_logger().error('OPENAI_API_KEY is not set')
            return

        secs = max(0.5, float(secs))
        wav_path = tmp_wav_path()

        try:
            print(f'recording for {secs:.1f}s...')
            data = sd.rec(
                int(secs * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float32',
            )
            sd.wait()
            save_wav_int16(wav_path, data, self.sample_rate, self.channels)
            print('transcribing...')
            txt = call_openai_transcribe_file(
                wav_path=wav_path,
                api_key=self.api_key,
                model=self.transcribe_model,
                base_url=self.base_url,
                timeout_sec=self.transcribe_timeout_sec,
            )
            if not txt:
                self.get_logger().error('transcription came back empty')
                return
            txt = clean_text(txt)
            print(f'transcript: {txt}')
            self._publish_text(txt)
        except Exception as e:
            self.get_logger().error(f'mic/transcribe failed: {e}')
        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def _handle_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return

        if line in ['/help', 'help']:
            self._print_help()
            return

        if line in ['/quit', 'quit', 'exit']:
            self.get_logger().info('stopping')
            rclpy.shutdown()
            return

        if line.startswith('/rec'):
            parts = line.split()
            secs = self.rec_secs
            if len(parts) > 1:
                try:
                    secs = float(parts[1])
                except Exception:
                    pass
            self._record_and_transcribe(secs)
            return

        self._publish_text(line)

    def _loop(self) -> None:
        while rclpy.ok() and not self._stop:
            try:
                line = input('cmd> ')
            except EOFError:
                time.sleep(0.2)
                continue
            except KeyboardInterrupt:
                rclpy.shutdown()
                return

            try:
                self._handle_line(line)
            except Exception as e:
                self.get_logger().error(f'bad console input: {e}')

    def destroy_node(self):
        self._stop = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
