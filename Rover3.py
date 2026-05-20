#!/usr/bin/env python3
"""
================================================================================
     MARMARA ROVER - JETSON ALL-IN-ONE v3 (BİRLEŞİK: ROVER + ROBOT KOL)
================================================================================

İki mod tek script'te — PC arayüzünden ZMQ üzerinden mod geçişi:
  - Manuel Kontrol : BLDC sürüş + RPM + step yönlendirme + ArUco + Drill
  - Robot Kol      : 5 eksen step + servo (startup lock, asimetrik rampa, watchdog)

Karşılıklı dışlayan (mutex): biri başlatılınca diğeri otomatik durur.
Kameralar (Logitech, RealSense) ve ArUco her iki modda da sürekli aktif.
Joystick her iki modda farklı yorumlanır; hiçbir mod aktif değilken işlenmez.

ZMQ KOMUTLARI (port 5560):
  {action:"start", launch:"manuel_control"} -> Rover modu aktif
  {action:"start", launch:"robot_kol"}      -> Kol modu aktif
  {action:"stop",  launch:"manuel_control"} -> Rover durdur
  {action:"stop",  launch:"robot_kol"}      -> Kol durdur (yumuşak rampa)
  {action:"stop_all"}                       -> Her ikisini durdur
  {action:"ping"}                           -> pong
  {action:"status"}                         -> her iki modun durumu

UART PORTLARI (varsayılan, CLI ile değiştirilebilir):
  --rover-bldc1  /dev/ttyUSB1  BLDC anakart 1
  --rover-bldc2  /dev/ttyUSB2  BLDC anakart 2
  --rover-step   /dev/ttyUSB0  Step motor Arduino
  --drill-port   /dev/ttyACM1  Drill mikrokontrolcü
  --arm-step     /dev/ttyUSB2  Robot kol step kartı
  --arm-servo    /dev/ttyUSB4  Robot kol servo kartı

TIP: Kalıcı port adlandırma için udev rule kullanın:
  /etc/udev/rules.d/99-rover.rules -> ATTRS{serial}=="..." SYMLINK+="rover_bldc1"

KULLANIM:
  python3 Rover3.py
  python3 Rover3.py --arm-step /dev/ttyUSB0 --arm-servo /dev/ttyUSB1
  python3 Rover3.py --no-aruco --no-realsense --no-drill
  python3 Rover3.py --aruco-source logitech --aruco-dict DICT_4X4_50

GEREKSINIMLER:
  pip3 install pyzmq pyserial opencv-python numpy
  # RealSense icin: pip3 install pyrealsense2
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, Imu, Joy
from std_msgs.msg import String, Float32, Int16, Header
from cv_bridge import CvBridge

import zmq, cv2, json, time, struct, sys, argparse
import numpy as np
from threading import Thread, Lock

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False
    print("[UYARI] pyserial yok -> pip3 install pyserial")

try:
    import pyrealsense2 as rs
    REALSENSE_OK = True
except ImportError:
    REALSENSE_OK = False
    print("[UYARI] pyrealsense2 yok -> pip3 install pyrealsense2")


# ═══════════════════════════════════════════════
#  ZMQ PORTLARI
# ═══════════════════════════════════════════════

LAUNCH_MANAGER_PORT = 5560
JOY_RECV_PORT       = 5556

ZMQ_PORTS = {
    'rscp':            5557,
    'step':            5561,
    'logitech':        6000,
    'telemetry':       6001,
    'realsense_rgb':   6002,
    'realsense_depth': 6003,
    'health':          6004,
    'aruco_debug':     6005,
    'aruco_detect':    6006,
    'drill_status':    6007,
}

# ═══════════════════════════════════════════════
#  ROVER MOTOR AYARLARI
# ═══════════════════════════════════════════════

START_FRAME   = 0xABCD
SERIAL_BAUD   = 115200
STEP_BAUD     = 115200
DRILL_BAUD    = 115200
COMMAND_FMT   = '<HhhH'
FEEDBACK_FMT  = '<HhhhhhhHH'
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FMT)  # 18 bytes

ROVER_AXIS_SPEED = 1
ROVER_AXIS_STEER = 2
ROVER_AXIS_RT    = 5
ROVER_BTN_LB     = 4
ROVER_BTN_RB     = 5

SPEED_MODES = {
    0: {'name': 'YAVAS', 'max': 100},
    1: {'name': 'ORTA',  'max': 200},
    2: {'name': 'HIZLI', 'max': 300},
}

STEER_MAX      = 400
STEER_DEADZONE = 0.3

# ═══════════════════════════════════════════════
#  ROBOT KOL AYARLARI
# ═══════════════════════════════════════════════

ARM_SERIAL_BAUD = 115200

ARM_AXIS_WRIST    = 0
ARM_AXIS_ELBOW    = 1
ARM_AXIS_WAIST    = 2
ARM_AXIS_SHOULDER = 3
ARM_AXIS_LT       = 4
ARM_AXIS_RT       = 5
ARM_BTN_X         = 2
ARM_BTN_Y         = 3

ARM_DEADZONE        = 0.15
WAIST_SPEED_MAX     = 2000
SHOULDER_SPEED_MAX  = 2000
ELBOW_SPEED_MAX     = 2000
WRIST_SPEED_MAX     = 2000
SPEED_SCALE         = 0.3

ACCEL_STEP            = 20.0
DECEL_STEP            = 8.0
JOY_TIMEOUT_SEC       = 0.8
STARTUP_HOLD_PACKETS  = 5
SHUTDOWN_RAMP_MAX_SEC = 4.0
ARM_SEND_INTERVAL_MS  = 50

GRIPPER_OPEN_ANGLE  = 180
GRIPPER_CLOSED_ANGLE = 0
GRIPPER_ROT_CENTER  = 90
GRIPPER_DEG_PER_SEC = 60.0


# ═══════════════════════════════════════════════
#  LOGITECH KAMERA NODE
# ═══════════════════════════════════════════════

class LogitechPublisherNode(Node):
    """Logitech C270 USB kameradan goruntu alip ROS topic'e publish eder."""

    def __init__(self):
        super().__init__('logitech_publisher_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 85)

        self.camera_id    = self.get_parameter('camera_id').value
        self.width        = self.get_parameter('width').value
        self.height       = self.get_parameter('height').value
        self.fps          = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.bridge        = CvBridge()
        self.cap           = None
        self.frame_count   = 0
        self.dropped_frames = 0

        self.pub_raw        = self.create_publisher(Image, '/logitech/image_raw', 10)
        self.pub_compressed = self.create_publisher(CompressedImage, '/logitech/image_compressed', 10)
        self.pub_info       = self.create_publisher(CameraInfo, '/logitech/camera_info', 10)

        self._init_camera()

        if self.cap and self.cap.isOpened():
            self.timer = self.create_timer(1.0 / self.fps, self._publish_frame)
            self.get_logger().info(
                f'Logitech HAZIR: {self.width}x{self.height} @ {self.fps}FPS (cam:{self.camera_id})'
            )
        else:
            self.get_logger().error('Logitech kamera acilamadi!')

    def _find_logitech_device(self):
        import subprocess, re
        try:
            out = subprocess.run(['v4l2-ctl', '--list-devices'],
                                 capture_output=True, text=True, timeout=3).stdout
            in_logi = False
            for line in out.splitlines():
                if any(k in line.lower() for k in ('c270', 'logitech', 'webcam c')):
                    in_logi = True
                elif in_logi and '/dev/video' in line:
                    m = re.search(r'/dev/video(\d+)', line)
                    if m:
                        return int(m.group(1))
                elif line and not line[0].isspace():
                    in_logi = False
        except Exception:
            pass
        return None

    def _init_camera(self):
        try:
            detected = self._find_logitech_device()
            if detected is not None and detected != self.camera_id:
                self.get_logger().info(f'Logitech otomatik bulundu: /dev/video{detected}')
                self.camera_id = detected

            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error(
                    f'Kamera acilamadi: /dev/video{self.camera_id} | '
                    f'Kontrol: "lsusb | grep 046d", "ls /dev/video*", "groups | grep video"'
                )
                self.cap = None
                return

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

            for _ in range(5):
                self.cap.read()

            ret, _ = self.cap.read()
            if not ret:
                self.get_logger().warn(
                    f'Kamera acildi ama ilk frame gelmiyor: /dev/video{self.camera_id}'
                )
        except Exception as e:
            self.get_logger().error(f'Kamera init hata: {e}')
            self.cap = None

    def _publish_frame(self):
        if not self.cap or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.dropped_frames += 1
            now = time.time()
            if not hasattr(self, '_last_drop_warn') or (now - self._last_drop_warn) >= 5.0:
                self._last_drop_warn = now
                self.get_logger().warn(
                    f'Logitech frame okunamiyor! Drop:{self.dropped_frames} '
                    f'(/dev/video{self.camera_id})'
                )
            return

        timestamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = timestamp
        msg.header.frame_id = 'logitech_optical_frame'
        self.pub_raw.publish(msg)

        _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        cmsg = CompressedImage()
        cmsg.header.stamp    = timestamp
        cmsg.header.frame_id = 'logitech_optical_frame'
        cmsg.format          = 'jpeg'
        cmsg.data            = jpg.tobytes()
        self.pub_compressed.publish(cmsg)

        info = CameraInfo()
        info.header.stamp    = timestamp
        info.header.frame_id = 'logitech_optical_frame'
        info.width           = self.width
        info.height          = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(self.width);  fy = float(self.height)
        cx = self.width / 2.0;  cy = self.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

        self.frame_count += 1
        if self.frame_count % 150 == 1:
            self.get_logger().info(f'Logitech aktif: {self.frame_count} frame gonderildi')

    def shutdown(self):
        if self.cap:
            self.cap.release()
            self.get_logger().info(f'Logitech kapatildi ({self.frame_count} frame)')


# ═══════════════════════════════════════════════
#  REALSENSE KAMERA NODE
# ═══════════════════════════════════════════════

class RealSensePublisherNode(Node):
    """Intel RealSense D435i kamerasından RGB, Depth, IMU publish eder."""

    def __init__(self):
        super().__init__('realsense_publisher_node')

        self.declare_parameter('rgb_width',   640)
        self.declare_parameter('rgb_height',  480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_imu', False)
        self.declare_parameter('align_depth_to_color', True)

        self.rgb_width    = self.get_parameter('rgb_width').value
        self.rgb_height   = self.get_parameter('rgb_height').value
        self.depth_width  = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.fps          = self.get_parameter('fps').value
        self.enable_imu   = self.get_parameter('enable_imu').value
        self.align_depth  = self.get_parameter('align_depth_to_color').value

        self.bridge      = CvBridge()
        self.pipeline    = None
        self.align       = None
        self.frame_count = {'rgb': 0, 'depth': 0}

        self.pub_rgb      = self.create_publisher(Image,      '/realsense/rgb/image_raw',    10)
        self.pub_rgb_info = self.create_publisher(CameraInfo, '/realsense/rgb/camera_info',  10)
        self.pub_depth    = self.create_publisher(Image,      '/realsense/depth/image_rect', 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, '/realsense/depth/camera_info', 10)

        if self.enable_imu:
            self.pub_imu_accel = self.create_publisher(Imu, '/realsense/imu/accel', 10)
            self.pub_imu_gyro  = self.create_publisher(Imu, '/realsense/imu/gyro',  10)

        self._init_realsense()

        if self.pipeline:
            self.timer = self.create_timer(1.0 / self.fps, self._publish_frames)
            self.get_logger().info(
                f'RealSense HAZIR: RGB {self.rgb_width}x{self.rgb_height}, '
                f'Depth {self.depth_width}x{self.depth_height} @ {self.fps}FPS'
            )
        else:
            self.get_logger().error('RealSense baslatilamadi!')

    def _init_realsense(self):
        if not REALSENSE_OK:
            self.get_logger().error('pyrealsense2 yuklenmemis!')
            return
        try:
            ctx     = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().error(
                    'RealSense cihazi bulunamadi! '
                    '"lsusb | grep -i intel" ile USB\'de gorunuyor mu kontrol edin.'
                )
                return

            device     = devices[0]
            serial_num = device.get_info(rs.camera_info.serial_number)
            name       = device.get_info(rs.camera_info.name)
            fw         = device.get_info(rs.camera_info.firmware_version)
            self.get_logger().info(f'RealSense bulundu: {name} (S/N:{serial_num}, FW:{fw})')

            self.pipeline = rs.pipeline()
            config        = rs.config()
            config.enable_device(serial_num)
            config.enable_stream(rs.stream.color, self.rgb_width,   self.rgb_height,   rs.format.rgb8, self.fps)
            config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16,  self.fps)

            if self.enable_imu:
                try:
                    config.enable_stream(rs.stream.accel)
                    config.enable_stream(rs.stream.gyro)
                except Exception as e:
                    self.get_logger().warn(f'IMU baslatilamadi: {e}')
                    self.enable_imu = False

            try:
                self.pipeline.start(config)
            except RuntimeError as e:
                self.get_logger().error(
                    f'RealSense pipeline.start() HATA: {e} | '
                    f'"rs-enumerate-devices" ile desteklenen cozunurlugu kontrol edin'
                )
                self.pipeline = None
                return

            if self.align_depth:
                self.align = rs.align(rs.stream.color)

            self.get_logger().info('RealSense stabilize ediliyor...')
            for _ in range(5):
                try:
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                except RuntimeError:
                    pass

            self.get_logger().info(
                f'RealSense pipeline hazir: '
                f'RGB {self.rgb_width}x{self.rgb_height}, '
                f'Depth {self.depth_width}x{self.depth_height} @ {self.fps}fps'
            )
        except Exception as e:
            self.get_logger().error(f'RealSense init hata: {e}')
            self.pipeline = None

    def _publish_frames(self):
        if not self.pipeline:
            return
        try:
            try:
                ok, frames = self.pipeline.try_wait_for_frames(timeout_ms=100)
                if not ok:
                    return
            except AttributeError:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)

            if self.align:
                frames = self.align.process(frames)

            timestamp = self.get_clock().now().to_msg()

            color_frame = frames.get_color_frame()
            if color_frame:
                rgb_img = np.asanyarray(color_frame.get_data())
                bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
                msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding='bgr8')
                msg.header.stamp    = timestamp
                msg.header.frame_id = 'realsense_color_optical_frame'
                self.pub_rgb.publish(msg)
                self.pub_rgb_info.publish(
                    self._make_camera_info(color_frame, timestamp, 'realsense_color_optical_frame')
                )
                self.frame_count['rgb'] += 1
                if self.frame_count['rgb'] % 150 == 1:
                    self.get_logger().info(f'RealSense RGB aktif: {self.frame_count["rgb"]} frame')

            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())
                msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
                msg.header.stamp    = timestamp
                msg.header.frame_id = 'realsense_depth_optical_frame'
                self.pub_depth.publish(msg)
                self.pub_depth_info.publish(
                    self._make_camera_info(depth_frame, timestamp, 'realsense_depth_optical_frame')
                )
                self.frame_count['depth'] += 1

            if self.enable_imu:
                accel = frames.first_or_default(rs.stream.accel)
                if accel:
                    ad   = accel.as_motion_frame().get_motion_data()
                    imsg = Imu()
                    imsg.header.stamp    = timestamp
                    imsg.header.frame_id = 'realsense_imu_frame'
                    imsg.linear_acceleration.x = float(ad.x)
                    imsg.linear_acceleration.y = float(ad.y)
                    imsg.linear_acceleration.z = float(ad.z)
                    self.pub_imu_accel.publish(imsg)
                gyro = frames.first_or_default(rs.stream.gyro)
                if gyro:
                    gd   = gyro.as_motion_frame().get_motion_data()
                    gmsg = Imu()
                    gmsg.header.stamp    = timestamp
                    gmsg.header.frame_id = 'realsense_imu_frame'
                    gmsg.angular_velocity.x = float(gd.x)
                    gmsg.angular_velocity.y = float(gd.y)
                    gmsg.angular_velocity.z = float(gd.z)
                    self.pub_imu_gyro.publish(gmsg)
        except Exception as e:
            self.get_logger().error(f'RealSense frame hata: {e}')

    def _make_camera_info(self, frame, timestamp, frame_id):
        intr = frame.profile.as_video_stream_profile().intrinsics
        msg  = CameraInfo()
        msg.header.stamp    = timestamp
        msg.header.frame_id = frame_id
        msg.width           = intr.width
        msg.height          = intr.height
        msg.distortion_model = 'plumb_bob'
        msg.d = [intr.coeffs[i] for i in range(5)]
        msg.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def shutdown(self):
        if self.pipeline:
            self.pipeline.stop()
            self.get_logger().info(
                f'RealSense kapatildi (RGB:{self.frame_count["rgb"]}, Depth:{self.frame_count["depth"]})'
            )


# ═══════════════════════════════════════════════
#  ZMQ BRIDGE NODE
# ═══════════════════════════════════════════════

class ZMQBridgeNode(Node):
    """PC ile ZMQ haberlesme: joystick alir, kamera/telemetri gonderir."""

    def __init__(self):
        super().__init__('zmq_bridge_node')
        self.declare_parameter('jpeg_quality', 85)
        self.jpeg_q = self.get_parameter('jpeg_quality').value
        self.ctx    = zmq.Context()

        self.pubs = {}
        for key, port in ZMQ_PORTS.items():
            s = self.ctx.socket(zmq.PUB)
            s.setsockopt(zmq.SNDHWM, 10)
            s.bind(f"tcp://*:{port}")
            self.pubs[key] = s
            self.get_logger().info(f'ZMQ PUB :{port} ({key})')

        self.joy_sub = self.ctx.socket(zmq.SUB)
        self.joy_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.joy_sub.setsockopt(zmq.RCVHWM,   5)
        self.joy_sub.setsockopt(zmq.CONFLATE,  1)
        self.joy_sub.setsockopt(zmq.RCVTIMEO, 100)
        self.joy_sub.bind(f"tcp://*:{JOY_RECV_PORT}")
        self.get_logger().info(f'ZMQ SUB :{JOY_RECV_PORT} (joystick)')

        self.bridge     = CvBridge()
        self.stats      = {}
        self.stats_lock = Lock()
        for k in list(ZMQ_PORTS.keys()) + ['joy']:
            self.stats[k + '_n'] = 0
            self.stats[k + '_t'] = 0.0
        self._last_joy_log = 0.0
        self._joy_count    = 0

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        self.create_subscription(Image,  '/logitech/image_raw',          self._cb_logi,         10)
        self.create_subscription(String, '/marmara/telemetry_json',      self._cb_telem,        10)
        self.create_subscription(Image,  '/realsense/rgb/image_raw',     self._cb_rgb,          10)
        self.create_subscription(Image,  '/realsense/depth/image_rect',  self._cb_depth,        10)
        self.create_subscription(String, '/marmara/rscp_json',           self._cb_rscp,         10)
        self.create_subscription(String, '/marmara/step_json',           self._cb_step,         10)
        self.create_subscription(Image,  '/aruco/debug_image',           self._cb_aruco,        10)
        self.create_subscription(String, '/aruco/detections',            self._cb_aruco_detect, 10)
        self.create_subscription(String, 'rover/drill_status',           self._cb_drill_status, 10)

        self.drill_cmd_pub = self.create_publisher(String, '/drill/command', 10)

        self.create_timer(1.0, self._pub_health)
        self.create_timer(5.0, self._log_stream_stats)
        self._stream_log_prev = {k: 0 for k in ZMQ_PORTS.keys()}

        self._joy_run = True
        self._joy_t   = Thread(target=self._joy_loop, daemon=True)
        self._joy_t.start()
        self.get_logger().info('ZMQ BRIDGE HAZIR')

    def _joy_loop(self):
        while self._joy_run:
            try:
                raw = self.joy_sub.recv_string()
            except zmq.Again:
                continue
            except Exception as e:
                self.get_logger().error(f'Joy SUB: {e}')
                time.sleep(0.1)
                continue
            try:
                data = json.loads(raw)
            except Exception:
                continue

            # Replay saldırısı önlemi: 2s'den eski veya geleceğe ait paketleri reddet
            msg_ts = data.get("timestamp")
            if msg_ts is not None:
                try:
                    age = abs(time.time() - float(msg_ts))
                    if age > 2.0:
                        self.get_logger().warn(f'Joy paket yaşı çok büyük: {age:.1f}s — reddedildi')
                        continue
                except (TypeError, ValueError):
                    pass

            msg_type = data.get("type", "")
            if msg_type == "joy":
                axes    = data.get("axes", [])
                buttons = data.get("buttons", [])
                ts      = data.get("timestamp", time.time())
                msg     = Joy()
                msg.header.stamp.sec     = int(ts)
                msg.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
                msg.header.frame_id      = "pc_joystick"
                msg.axes    = [float(a) for a in axes]
                msg.buttons = [int(b)   for b in buttons]
                self.joy_pub.publish(msg)
                self._joy_count += 1
                with self.stats_lock:
                    self.stats['joy_n'] += 1
                    self.stats['joy_t']  = time.time()
                now = time.time()
                if (now - self._last_joy_log) >= 0.2:
                    self._last_joy_log = now
                    spd   = axes[ROVER_AXIS_SPEED] if len(axes) > ROVER_AXIS_SPEED else 0
                    steer = axes[ROVER_AXIS_STEER] if len(axes) > ROVER_AXIS_STEER else 0
                    self.get_logger().info(f'JOY#{self._joy_count} spd:{spd:+.2f} str:{steer:+.2f}')

            elif msg_type == "keyboard":
                key = data.get("key", "").lower()
                if key in ('z', 'x', 'c', 'v'):
                    cmd_msg      = String()
                    cmd_msg.data = json.dumps({'key': key, 'timestamp': time.time()})
                    self.drill_cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f'DRILL klavye komutu alindi: {key}')

    def _send_img(self, ros_msg, key):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(ros_msg, desired_encoding='bgr8')
            _, jpg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            meta   = json.dumps({
                'timestamp': ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec * 1e-9,
                'width':     ros_msg.width,
                'height':    ros_msg.height,
            }).encode()
            self.pubs[key].send_multipart([meta, jpg.tobytes()])
            with self.stats_lock:
                self.stats[key + '_n'] += 1
                self.stats[key + '_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'{key}: {e}')

    def _cb_logi(self, msg):   self._send_img(msg, 'logitech')
    def _cb_rgb(self,  msg):   self._send_img(msg, 'realsense_rgb')
    def _cb_aruco(self, msg):  self._send_img(msg, 'aruco_debug')

    def _cb_depth(self, msg):
        try:
            d          = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            valid_mask = d > 0
            d_float    = d.astype(np.float32)
            np.clip(d_float, 100.0, 5000.0, out=d_float)
            d_norm = np.zeros(d.shape, dtype=np.uint8)
            if valid_mask.any():
                d_norm[valid_mask] = (
                    (d_float[valid_mask] - 100.0) / (5000.0 - 100.0) * 255.0
                ).astype(np.uint8)
            cm             = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)
            cm[~valid_mask] = 0
            _, jpg = cv2.imencode('.jpg', cm, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            meta   = json.dumps({
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'width':     msg.width,
                'height':    msg.height,
            }).encode()
            self.pubs['realsense_depth'].send_multipart([meta, jpg.tobytes()])
            with self.stats_lock:
                self.stats['realsense_depth_n'] += 1
                self.stats['realsense_depth_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'depth: {e}')

    def _cb_telem(self, msg):
        try:
            d = json.loads(msg.data)
            d['bridge_ts'] = time.time()
            self.pubs['telemetry'].send_string(json.dumps(d))
            with self.stats_lock:
                self.stats['telemetry_n'] += 1
                self.stats['telemetry_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'telem: {e}')

    def _cb_rscp(self, msg):
        try:
            d = json.loads(msg.data)
            self.pubs['rscp'].send_string(json.dumps(d))
            with self.stats_lock:
                self.stats['rscp_n'] += 1
                self.stats['rscp_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'rscp: {e}')

    def _cb_step(self, msg):
        try:
            d = json.loads(msg.data)
            self.pubs['step'].send_string(json.dumps(d))
            with self.stats_lock:
                self.stats['step_n'] += 1
                self.stats['step_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'step: {e}')

    def _cb_drill_status(self, msg):
        try:
            d = json.loads(msg.data)
            self.pubs['drill_status'].send_string(json.dumps(d))
            with self.stats_lock:
                self.stats['drill_status_n'] += 1
                self.stats['drill_status_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'drill_status: {e}')

    def _cb_aruco_detect(self, msg):
        try:
            d = json.loads(msg.data)
            self.pubs['aruco_detect'].send_string(json.dumps(d))
            with self.stats_lock:
                self.stats['aruco_detect_n'] += 1
                self.stats['aruco_detect_t']  = time.time()
        except Exception as e:
            self.get_logger().error(f'aruco_detect: {e}')

    def _pub_health(self):
        now = time.time()
        with self.stats_lock:
            h = {'timestamp': now, 'status': 'healthy', 'streams': {}}
            for key in list(ZMQ_PORTS.keys()) + ['joy']:
                h['streams'][key] = {
                    'alive': (now - self.stats.get(key + '_t', 0)) < 2.0,
                    'count': self.stats.get(key + '_n', 0),
                }
        try:
            self.pubs['health'].send_string(json.dumps(h))
        except Exception:
            pass

    def _log_stream_stats(self):
        with self.stats_lock:
            parts = []
            for key in ('logitech', 'realsense_rgb', 'realsense_depth', 'aruco_debug'):
                total = self.stats.get(key + '_n', 0)
                delta = total - self._stream_log_prev.get(key, 0)
                self._stream_log_prev[key] = total
                parts.append(f'{key}:{delta}/5s')
        self.get_logger().info('ZMQ frame sayaci: ' + '  '.join(parts))

    def shutdown(self):
        self._joy_run = False
        if self._joy_t.is_alive():
            self._joy_t.join(timeout=2.0)
        self.joy_sub.close()
        for s in self.pubs.values():
            s.close()
        self.ctx.term()


# ═══════════════════════════════════════════════
#  ARUCO TESPİT NODE
# ═══════════════════════════════════════════════

class ArucoDetectionNode(Node):
    """RealSense veya Logitech RGB goruntusunden ArUco marker tespit eder."""

    def __init__(self, image_topic='/realsense/rgb/image_raw', dictionary='DICT_5X5_250'):
        super().__init__('aruco_detection_node')

        self.declare_parameter('image_topic',        image_topic)
        self.declare_parameter('debug_image_topic',  '/aruco/debug_image')
        self.declare_parameter('detections_topic',   '/aruco/detections')
        self.declare_parameter('aruco_dictionary',   dictionary)
        self.declare_parameter('min_marker_id',      -1)
        self.declare_parameter('max_marker_id',      -1)
        self.declare_parameter('log_interval_sec',   1.0)

        self.image_topic        = self.get_parameter('image_topic').value
        self.debug_image_topic  = self.get_parameter('debug_image_topic').value
        self.detections_topic   = self.get_parameter('detections_topic').value
        self.min_marker_id      = int(self.get_parameter('min_marker_id').value)
        self.max_marker_id      = int(self.get_parameter('max_marker_id').value)
        self.log_interval_sec   = float(self.get_parameter('log_interval_sec').value)

        dict_name = self.get_parameter('aruco_dictionary').value
        dict_id   = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_5X5_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params    = cv2.aruco.DetectorParameters()
            self.aruco_detector  = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_params   = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None

        self.bridge        = CvBridge()
        self.last_log_time = 0.0

        self.debug_pub      = self.create_publisher(Image,  self.debug_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic,  10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.get_logger().info(f'ArUco HAZIR: {self.image_topic} -> {self.debug_image_topic}')

    def _image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge hata: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        detections = self._build_detections(corners, ids, cv_image.shape[1])
        self._draw_overlay(cv_image, detections)
        self._publish_debug(cv_image, msg)
        self._publish_detections(detections, msg)
        self._log(detections)

    def _build_detections(self, corners, ids, image_width):
        if ids is None:
            return []
        result = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if not self._id_allowed(marker_id):
                continue
            pts = marker_corners[0]
            cx  = float(np.mean(pts[:, 0]))
            cy  = float(np.mean(pts[:, 1]))
            result.append({
                'id':                 marker_id,
                'center_x':           cx,
                'center_y':           cy,
                'area':               float(cv2.contourArea(pts.astype(np.float32))),
                'normalized_error_x': (cx - image_width / 2.0) / (image_width / 2.0),
                'corners':            pts.tolist(),
            })
        result.sort(key=lambda d: d['area'], reverse=True)
        return result

    def _id_allowed(self, marker_id):
        if self.min_marker_id >= 0 and marker_id < self.min_marker_id:
            return False
        if self.max_marker_id >= 0 and marker_id > self.max_marker_id:
            return False
        return True

    def _draw_overlay(self, cv_image, detections):
        if detections:
            valid_ids     = np.array([[d['id']] for d in detections])
            valid_corners = [np.array([d['corners']], dtype=np.float32) for d in detections]
            cv2.aruco.drawDetectedMarkers(cv_image, valid_corners, valid_ids)
        for d in detections:
            center = (int(d['center_x']), int(d['center_y']))
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
            cv2.putText(cv_image, f"ID {d['id']}", (center[0] + 10, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        status = 'Aruco detected' if detections else 'No Aruco marker'
        cv2.rectangle(cv_image, (10, 10), (330, 72), (0, 0, 0), -1)
        cv2.rectangle(cv_image, (10, 10), (330, 72), (0, 255, 0), 2)
        cv2.putText(cv_image, status, (22, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image, f'Count: {len(detections)}', (22, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    def _publish_debug(self, cv_image, source_msg):
        try:
            dmsg        = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            dmsg.header = source_msg.header
            self.debug_pub.publish(dmsg)
        except Exception as e:
            self.get_logger().error(f'ArUco debug publish hata: {e}')

    def _publish_detections(self, detections, source_msg):
        stamp   = source_msg.header.stamp
        payload = {
            'timestamp':  stamp.sec + stamp.nanosec * 1e-9,
            'frame_id':   source_msg.header.frame_id,
            'count':      len(detections),
            'detections': detections,
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self.detections_pub.publish(msg)

    def _log(self, detections):
        now = time.time()
        if now - self.last_log_time < self.log_interval_sec:
            return
        self.last_log_time = now
        if not detections:
            self.get_logger().info('ArUco: marker yok.')
        else:
            ids = ', '.join(str(d['id']) for d in detections)
            self.get_logger().info(f'ArUco: Tespit edilen ID -> {ids}')

    def shutdown(self):
        pass


# ═══════════════════════════════════════════════
#  DRILL MEKANIZMASI NODE
#  active=False iken komutları yok sayar.
#  activate() portu acar, deactivate() motorları durdurur ve portu kapatır.
# ═══════════════════════════════════════════════

class DrillControlNode(Node):
    """
    Drill mekanizması kontrolü.
    Manuel mod aktifken activate() çağrılır; mod kapanınca deactivate().

    Motor 1 (z/x tuşları): z -> saga don toggle, x -> sola don toggle
    Motor 2 (c/v tuşları): c -> saga don toggle, v -> sola don toggle

    Mikrokontrolcüye gönderilen format: {key,value}\n  örn: {z,1}\n
    """

    def __init__(self, drill_port='/dev/ttyUSB3', drill_baud=DRILL_BAUD):
        super().__init__('drill_control_node')
        self.declare_parameter('drill_port', drill_port)
        self.declare_parameter('drill_baud', drill_baud)

        self.drill_port = self.get_parameter('drill_port').value
        self.drill_baud = self.get_parameter('drill_baud').value

        self.motor1_state = None  # None / 'right' / 'left'
        self.motor2_state = None

        self.ser_drill = None
        self.active    = False

        self.pub_drill_status = self.create_publisher(String, 'rover/drill_status', 10)
        self.create_subscription(String, '/drill/command', self._drill_cmd_cb, 10)

        self.get_logger().info(
            f'DRILL NODE HAZIR (pasif) | UART:{self.drill_port} | '
            f'z/x=Motor1(sag/sol) c/v=Motor2(sag/sol)'
        )

    # ─────────────── UART ───────────────

    def _open_drill_serial(self):
        try:
            self.ser_drill = serial.Serial(
                self.drill_port, self.drill_baud,
                bytesize=8, parity='N', stopbits=1, timeout=0.0,
            )
            self.get_logger().info(f'DRILL UART OK: {self.drill_port} @ {self.drill_baud}')
        except Exception as e:
            self.get_logger().error(f'DRILL UART HATA: {e}')
            self.ser_drill = None

    def _drill_send_raw(self, key, value):
        cmd = '{' + f'{key},{value}' + '}\n'
        if self.ser_drill and self.ser_drill.is_open:
            try:
                self.ser_drill.write(cmd.encode('ascii'))
                self.get_logger().info(f'DRILL gonderildi: {cmd.strip()}')
            except Exception as e:
                self.get_logger().error(f'DRILL yazma hatasi: {e}')
                try:
                    self.ser_drill = serial.Serial(
                        self.drill_port, self.drill_baud,
                        bytesize=8, parity='N', stopbits=1, timeout=0.0,
                    )
                except Exception:
                    self.ser_drill = None
        else:
            self.get_logger().warn(f'DRILL UART bagli degil! Komut: {cmd.strip()}')

    # ─────────────── LIFECYCLE ───────────────

    def activate(self):
        if self.active:
            return
        self.get_logger().info('>>> DRILL AKTIVASYON...')
        if SERIAL_OK:
            self._open_drill_serial()
        self.active = True
        self.get_logger().info(f'>>> DRILL AKTIF ({self.drill_port})')

    def deactivate(self):
        if not self.active:
            return
        self.active = False
        self.get_logger().info('>>> DRILL DURDURULÜYOR...')
        self._drill_send_raw('z', 0)
        self._drill_send_raw('x', 0)
        self._drill_send_raw('c', 0)
        self._drill_send_raw('v', 0)
        self.motor1_state = None
        self.motor2_state = None
        if self.ser_drill and self.ser_drill.is_open:
            self.ser_drill.close()
        self.ser_drill = None
        self.get_logger().info('>>> DRILL PASIF (port kapali)')

    def shutdown(self):
        self.deactivate()

    # ─────────────── CALLBACK ───────────────

    def _drill_cmd_cb(self, msg):
        if not self.active:
            return
        try:
            data = json.loads(msg.data)
            key  = data.get('key', '').lower()
            self._handle_key(key)
        except Exception as e:
            self.get_logger().error(f'Drill komut parse hatasi: {e}')

    def _handle_key(self, ch):
        if ch == 'z':
            if self.motor1_state == 'right':
                self._drill_send_raw('z', 0)
                self.motor1_state = None
                self.get_logger().info('DRILL Motor1 SAGA DON -> DURDURULDU')
            else:
                if self.motor1_state == 'left':
                    self._drill_send_raw('x', 0)
                self._drill_send_raw('z', 1)
                self.motor1_state = 'right'
                self.get_logger().info('DRILL Motor1 -> SAGA DON')
            self._publish_status()
        elif ch == 'x':
            if self.motor1_state == 'left':
                self._drill_send_raw('x', 0)
                self.motor1_state = None
                self.get_logger().info('DRILL Motor1 SOLA DON -> DURDURULDU')
            else:
                if self.motor1_state == 'right':
                    self._drill_send_raw('z', 0)
                self._drill_send_raw('x', 1)
                self.motor1_state = 'left'
                self.get_logger().info('DRILL Motor1 -> SOLA DON')
            self._publish_status()
        elif ch == 'c':
            if self.motor2_state == 'right':
                self._drill_send_raw('c', 0)
                self.motor2_state = None
                self.get_logger().info('DRILL Motor2 SAGA DON -> DURDURULDU')
            else:
                if self.motor2_state == 'left':
                    self._drill_send_raw('v', 0)
                self._drill_send_raw('c', 1)
                self.motor2_state = 'right'
                self.get_logger().info('DRILL Motor2 -> SAGA DON')
            self._publish_status()
        elif ch == 'v':
            if self.motor2_state == 'left':
                self._drill_send_raw('v', 0)
                self.motor2_state = None
                self.get_logger().info('DRILL Motor2 SOLA DON -> DURDURULDU')
            else:
                if self.motor2_state == 'right':
                    self._drill_send_raw('c', 0)
                self._drill_send_raw('v', 1)
                self.motor2_state = 'left'
                self.get_logger().info('DRILL Motor2 -> SOLA DON')
            self._publish_status()

    def _publish_status(self):
        msg      = String()
        m1_str   = self.motor1_state.upper() if self.motor1_state else 'STOP'
        m2_str   = self.motor2_state.upper() if self.motor2_state else 'STOP'
        msg.data = json.dumps({
            'motor1_state': self.motor1_state,
            'motor2_state': self.motor2_state,
            'motor1_str':   m1_str,
            'motor2_str':   m2_str,
            'timestamp':    time.time(),
        })
        self.pub_drill_status.publish(msg)


# ═══════════════════════════════════════════════
#  MANUEL KONTROL NODE
#  active=False iken joy/send/recv callback'leri çalışmaz, UART kapalı.
#  activate() portu açar, deactivate() motorları durdurur ve portu kapatır.
# ═══════════════════════════════════════════════

class ManuelControlNode(Node):
    """
    /joy -> UART motor komut + Hoverboard'dan RPM okuma.

    UART1 (rover-bldc1): BLDC anakart 1 - sadece hız (steer=0)
    UART2 (rover-bldc2): BLDC anakart 2 - sadece hız (steer=0)
    UART3 (rover-step):  Step motor Arduino - LEFT / RIGHT / STOP
    """

    def __init__(self,
                 serial_port='/dev/ttyUSB1',
                 serial_port2='/dev/ttyUSB5',
                 serial_port3='/dev/ttyUSB0'):
        super().__init__('manuel_control_node')
        self.declare_parameter('serial_port',     serial_port)
        self.declare_parameter('serial_port2',    serial_port2)
        self.declare_parameter('serial_port3',    serial_port3)
        self.declare_parameter('baud_rate',       SERIAL_BAUD)
        self.declare_parameter('baud_rate3',      STEP_BAUD)
        self.declare_parameter('send_interval_ms', 100)
        self.declare_parameter('deadzone',         0.1)
        self.declare_parameter('brake_decel_rate', 20.0)

        self.serial_port  = self.get_parameter('serial_port').value
        self.serial_port2 = self.get_parameter('serial_port2').value
        self.serial_port3 = self.get_parameter('serial_port3').value
        self.baud_rate    = self.get_parameter('baud_rate').value
        self.baud_rate3   = self.get_parameter('baud_rate3').value
        self.send_ms      = self.get_parameter('send_interval_ms').value
        self.deadzone     = self.get_parameter('deadzone').value
        self.brake_decel  = self.get_parameter('brake_decel_rate').value

        self.speed_mode   = 1
        self.target_speed = 0
        self.target_steer = 0
        self.cur_speed    = 0.0
        self.cur_steer    = 0.0
        self.is_braking   = False
        self.brake_pct    = 0.0
        self.prev_rb      = 0
        self.prev_lb      = 0
        self.bat_v        = 0.0
        self.bat_v2       = 0.0
        self.board_temp   = 0.0
        self.board_temp2  = 0.0
        self._last_log    = 0.0
        self._joy_received = False
        self._joy_count   = 0

        self.rpm_right  = 0
        self.rpm_left   = 0
        self.rpm_right2 = 0
        self.rpm_left2  = 0

        self._prev1       = None;  self._collecting1 = False;  self._buf1 = bytearray()
        self._prev2       = None;  self._collecting2 = False;  self._buf2 = bytearray()
        self._start_lo    = START_FRAME & 0xFF
        self._start_hi    = (START_FRAME >> 8) & 0xFF
        self.feedback_count1 = 0
        self.feedback_count2 = 0

        self._step_state = "STOP"
        self._step_buf   = ""

        # UART nesneleri activate()'de açılır
        self.ser  = None
        self.ser2 = None
        self.ser3 = None
        self.active = False

        self.create_subscription(Joy, 'joy', self._joy_cb, 10)

        self.pub_status  = self.create_publisher(String,  'rover/status',          10)
        self.pub_mode    = self.create_publisher(String,  'rover/speed_mode',      10)
        self.pub_bat     = self.create_publisher(Float32, 'rover/battery_voltage',  10)
        self.pub_bat2    = self.create_publisher(Float32, 'rover/battery_voltage2', 10)
        self.pub_spd     = self.create_publisher(Int16,   'rover/cmd_speed',       10)
        self.pub_str     = self.create_publisher(Int16,   'rover/cmd_steer',       10)
        self.pub_step    = self.create_publisher(String,  'rover/step_dir',        10)
        self.pub_rpm_right  = self.create_publisher(Int16, 'rover/rpm_right',  10)
        self.pub_rpm_left   = self.create_publisher(Int16, 'rover/rpm_left',   10)
        self.pub_rpm_right2 = self.create_publisher(Int16, 'rover/rpm_right2', 10)
        self.pub_rpm_left2  = self.create_publisher(Int16, 'rover/rpm_left2',  10)
        self.pub_telemetry  = self.create_publisher(String, '/marmara/telemetry_json', 10)

        self.create_timer(self.send_ms / 1000.0, self._send_cb)
        self.create_timer(0.02, self._recv_cb)  # 50 Hz

        self.get_logger().info(
            f'MANUEL KONTROL NODE HAZIR (pasif) | '
            f'UART1:{self.serial_port} UART2:{self.serial_port2} STEP:{self.serial_port3}'
        )

    # ─────────────── UART ───────────────

    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate,
                bytesize=8, parity='N', stopbits=1, timeout=0.0,
            )
            self.get_logger().info(f'UART1 OK: {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'UART1 HATA: {e}')
            self.ser = None
        try:
            self.ser2 = serial.Serial(
                self.serial_port2, self.baud_rate,
                bytesize=8, parity='N', stopbits=1, timeout=0.0,
            )
            self.get_logger().info(f'UART2 OK: {self.serial_port2}')
        except Exception as e:
            self.get_logger().error(f'UART2 HATA: {e}')
            self.ser2 = None
        try:
            self.ser3 = serial.Serial(
                self.serial_port3, self.baud_rate3,
                bytesize=8, parity='N', stopbits=1, timeout=0.0,
            )
            self.get_logger().info(f'STEP UART OK: {self.serial_port3}')
        except Exception as e:
            self.get_logger().error(f'STEP UART HATA: {e}')
            self.ser3 = None

    def _close_serial(self):
        for attr in ('ser', 'ser2', 'ser3'):
            s = getattr(self, attr)
            if s and s.is_open:
                try:
                    s.close()
                except Exception:
                    pass
            setattr(self, attr, None)

    # ─────────────── LIFECYCLE ───────────────

    def activate(self):
        if self.active:
            return
        self.get_logger().info('>>> MANUEL KONTROL AKTIVASYON BAŞLADI...')
        if SERIAL_OK:
            self._open_serial()
        self._joy_received = False
        self._joy_count    = 0
        self.cur_speed     = 0.0
        self.cur_steer     = 0.0
        self.active        = True
        mode = SPEED_MODES[self.speed_mode]
        self._pub_mode()
        self.get_logger().info(f'>>> MANUEL KONTROL AKTIF | Mod:{mode["name"]}')

    def deactivate(self):
        if not self.active:
            return
        self.active = False  # Önce flag'i kapat (_send_cb/_joy_cb erken döner)
        self.get_logger().info('>>> MANUEL KONTROL DURDURULÜYOR...')
        self._uart_send(0, 0)
        time.sleep(0.05)
        self._uart_send(0, 0)
        self._step_send("STOP")
        self._close_serial()
        self.get_logger().info('>>> MANUEL KONTROL PASIF (portlar kapali)')

    def shutdown(self):
        self.deactivate()

    # ─────────────── UART I/O ───────────────

    def _uart_send(self, steer, speed):
        steer = max(-1000, min(1000, int(steer)))
        speed = max(-1000, min(1000, int(speed)))
        chk = START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
        pkt = struct.pack(COMMAND_FMT, START_FRAME, steer, speed, chk & 0xFFFF)
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(pkt)
            except serial.SerialException as e:
                self.get_logger().error(f'BLDC1 yazma hatası ({self.serial_port}): {e}')
                try:
                    self.ser = serial.Serial(
                        self.serial_port, self.baud_rate,
                        bytesize=8, parity='N', stopbits=1, timeout=0.0,
                    )
                    self.get_logger().warn(f'BLDC1 yeniden açıldı: {self.serial_port}')
                except serial.SerialException as re:
                    self.get_logger().error(
                        f'BLDC1 yeniden açılamadı ({self.serial_port}): {re} — MOTOR KOMUTU GÖNDERİLEMEDİ'
                    )
                    self.ser = None
        if self.ser2 and self.ser2.is_open:
            try:
                self.ser2.write(pkt)
            except serial.SerialException as e:
                self.get_logger().error(f'BLDC2 yazma hatası ({self.serial_port2}): {e}')
                try:
                    self.ser2 = serial.Serial(
                        self.serial_port2, self.baud_rate,
                        bytesize=8, parity='N', stopbits=1, timeout=0.0,
                    )
                    self.get_logger().warn(f'BLDC2 yeniden açıldı: {self.serial_port2}')
                except serial.SerialException as re:
                    self.get_logger().error(
                        f'BLDC2 yeniden açılamadı ({self.serial_port2}): {re} — MOTOR KOMUTU GÖNDERİLEMEDİ'
                    )
                    self.ser2 = None

    def _step_send(self, cmd):
        if self.ser3 and self.ser3.is_open:
            try:
                self.ser3.write((cmd + '\n').encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().error(f'STEP yazma hatası ({self.serial_port3}): {e}')
                try:
                    self.ser3 = serial.Serial(
                        self.serial_port3, self.baud_rate3,
                        bytesize=8, parity='N', stopbits=1, timeout=0.0,
                    )
                    self.get_logger().warn(f'STEP yeniden açıldı: {self.serial_port3}')
                except serial.SerialException as re:
                    self.get_logger().error(
                        f'STEP yeniden açılamadı ({self.serial_port3}): {re} — STEP KOMUTU GÖNDERİLEMEDİ'
                    )
                    self.ser3 = None

    def _step_recv(self):
        if not self.ser3 or not self.ser3.is_open:
            return
        try:
            while self.ser3.in_waiting > 0:
                c = self.ser3.read(1).decode('ascii', errors='ignore')
                if c == '\n':
                    line = self._step_buf.strip()
                    self._step_buf = ""
                    if line:
                        self.get_logger().debug(f'STEP yanit: {line}')
                else:
                    self._step_buf += c
        except Exception:
            pass

    def _read_feedback(self, ser_obj, board_num):
        if not ser_obj or not ser_obj.is_open:
            return
        if board_num == 1:
            _prev = self._prev1;  _collecting = self._collecting1;  _buf = self._buf1
        else:
            _prev = self._prev2;  _collecting = self._collecting2;  _buf = self._buf2

        try:
            data = ser_obj.read(256)
            if not data:
                return
            for b in data:
                if _prev is None:
                    _prev = b
                    continue
                if (not _collecting) and (_prev == self._start_lo) and (b == self._start_hi):
                    _collecting = True
                    _buf = bytearray()
                    _buf.append(_prev)
                    _buf.append(b)
                elif _collecting:
                    _buf.append(b)
                    if len(_buf) == FEEDBACK_SIZE:
                        fields = struct.unpack(FEEDBACK_FMT, bytes(_buf))
                        start      = fields[0]
                        recv_csum  = fields[-1]
                        calc_csum  = (start ^
                                      (fields[1] & 0xFFFF) ^ (fields[2] & 0xFFFF) ^
                                      (fields[3] & 0xFFFF) ^ (fields[4] & 0xFFFF) ^
                                      (fields[5] & 0xFFFF) ^ (fields[6] & 0xFFFF) ^
                                      (fields[7] & 0xFFFF)) & 0xFFFF
                        _collecting = False
                        _buf        = bytearray()
                        if start == START_FRAME and calc_csum == recv_csum:
                            speedR = fields[3]
                            speedL = fields[4]
                            batV   = fields[5] / 100.0
                            temp   = fields[6] / 10.0
                            if board_num == 1:
                                self.feedback_count1 += 1
                                self.rpm_right = speedR;  self.rpm_left = speedL
                                self.bat_v     = batV;    self.board_temp = temp
                                m = Int16();  m.data = speedR;  self.pub_rpm_right.publish(m)
                                m = Int16();  m.data = speedL;  self.pub_rpm_left.publish(m)
                                m = Float32(); m.data = batV;   self.pub_bat.publish(m)
                            else:
                                self.feedback_count2 += 1
                                self.rpm_right2 = speedR;  self.rpm_left2 = speedL
                                self.bat_v2     = batV;    self.board_temp2 = temp
                                m = Int16();  m.data = speedR;  self.pub_rpm_right2.publish(m)
                                m = Int16();  m.data = speedL;  self.pub_rpm_left2.publish(m)
                                m = Float32(); m.data = batV;   self.pub_bat2.publish(m)
                _prev = b

            if board_num == 1:
                self._prev1 = _prev;  self._collecting1 = _collecting;  self._buf1 = _buf
            else:
                self._prev2 = _prev;  self._collecting2 = _collecting;  self._buf2 = _buf
        except Exception as e:
            self.get_logger().warn(f'UART{board_num} feedback hata: {e}')

    # ─────────────── CALLBACKS ───────────────

    def _joy_cb(self, msg):
        if not self.active:
            return
        self._joy_count += 1
        if not self._joy_received:
            self._joy_received = True
            self.get_logger().info(f'>>> ILK JOY ALINDI! axes={len(msg.axes)} btn={len(msg.buttons)}')

        raw_speed = msg.axes[ROVER_AXIS_SPEED] if ROVER_AXIS_SPEED < len(msg.axes) else 0.0
        raw_steer = msg.axes[ROVER_AXIS_STEER] if ROVER_AXIS_STEER < len(msg.axes) else 0.0
        raw_rt    = msg.axes[ROVER_AXIS_RT]    if ROVER_AXIS_RT    < len(msg.axes) else -1.0
        speed_in  = self._dz(raw_speed)
        rb = msg.buttons[ROVER_BTN_RB] if ROVER_BTN_RB < len(msg.buttons) else 0
        lb = msg.buttons[ROVER_BTN_LB] if ROVER_BTN_LB < len(msg.buttons) else 0

        if rb == 1 and self.prev_rb == 0 and self.speed_mode < 2:
            self.speed_mode += 1
            self._pub_mode()
            self.get_logger().info(f'HIZ MODU -> {SPEED_MODES[self.speed_mode]["name"]}')
        if lb == 1 and self.prev_lb == 0 and self.speed_mode > 0:
            self.speed_mode -= 1
            self._pub_mode()
            self.get_logger().info(f'HIZ MODU -> {SPEED_MODES[self.speed_mode]["name"]}')
        self.prev_rb = rb
        self.prev_lb = lb

        self.brake_pct = max(0.0, min(1.0, (raw_rt + 1.0) / 2.0))
        self.is_braking = self.brake_pct > 0.05
        max_spd = SPEED_MODES[self.speed_mode]['max']

        if self.is_braking:
            self.target_speed = 0
            self.target_steer = 0
        else:
            self.target_speed = int(speed_in * max_spd)
            self.target_steer = 0

        if abs(raw_steer) > STEER_DEADZONE:
            new_step = "RIGHT" if raw_steer < 0 else "LEFT"
        else:
            new_step = "STOP"

        if new_step != self._step_state:
            self._step_state = new_step
            self._step_send(new_step)
            self.get_logger().info(f'STEP -> {new_step}')

    def _send_cb(self):
        if not self.active:
            return
        if not self._joy_received:
            return

        if self.is_braking:
            if abs(self.cur_speed) > 1.0:
                self.cur_speed += (-self.brake_decel if self.cur_speed > 0 else self.brake_decel)
                self.cur_speed = (max(0.0, self.cur_speed) if self.cur_speed > 0
                                  else min(0.0, self.cur_speed))
            else:
                self.cur_speed = 0.0
        else:
            self.cur_speed = float(self.target_speed)
        self.cur_steer = float(self.target_steer)

        s_spd = int(self.cur_speed)
        s_str = int(self.cur_steer)
        self._uart_send(s_str, s_spd)

        m = Int16();  m.data = s_spd;  self.pub_spd.publish(m)
        m = Int16();  m.data = s_str;  self.pub_str.publish(m)
        m = String(); m.data = self._step_state;  self.pub_step.publish(m)

        mode = SPEED_MODES[self.speed_mode]
        brk  = " FREN" if self.is_braking else ""
        m    = String()
        m.data = f'Mod:{mode["name"]} Hiz:{s_spd} Step:{self._step_state}{brk}'
        self.pub_status.publish(m)

        telem = {
            'timestamp':  time.time(),
            'type':       'manuel',
            'speed_mode': mode['name'],
            'max_speed':  mode['max'],
            'cmd_speed':  s_spd,
            'cmd_steer':  s_str,
            'step_state': self._step_state,
            'braking':    self.is_braking,
            'brake_percent': self.brake_pct,
            'battery': {
                'board1_voltage': self.bat_v,
                'board2_voltage': self.bat_v2,
            },
            'temperature': {
                'board1_temp': self.board_temp,
                'board2_temp': self.board_temp2,
            },
            'rpm': {
                'board1_right': self.rpm_right,
                'board1_left':  self.rpm_left,
                'board2_right': self.rpm_right2,
                'board2_left':  self.rpm_left2,
            },
            'feedback_count': {
                'board1': self.feedback_count1,
                'board2': self.feedback_count2,
            },
        }
        telem_msg      = String()
        telem_msg.data = json.dumps(telem)
        self.pub_telemetry.publish(telem_msg)

        now = time.time()
        if (now - self._last_log) >= 0.2:
            self._last_log = now
            bat     = f"Bat:{self.bat_v:.1f}V/{self.bat_v2:.1f}V" if (self.bat_v > 0 or self.bat_v2 > 0) else "Bat:--V"
            rpm_str = f"RPM[R1:{self.rpm_right:+5d} L1:{self.rpm_left:+5d} | R2:{self.rpm_right2:+5d} L2:{self.rpm_left2:+5d}]"
            fb_str  = f"FB:{self.feedback_count1}/{self.feedback_count2}"
            self.get_logger().info(
                f'[{mode["name"]}] Hiz:{s_spd:+4d} Step:{self._step_state}{brk} | {bat} | {rpm_str} | {fb_str}'
            )

    def _recv_cb(self):
        if not self.active:
            return
        self._read_feedback(self.ser,  1)
        self._read_feedback(self.ser2, 2)
        self._step_recv()

    def _dz(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        s = 1.0 if v > 0 else -1.0
        return s * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _pub_mode(self):
        m      = String()
        mode   = SPEED_MODES[self.speed_mode]
        m.data = f'{mode["name"]}:{mode["max"]}'
        self.pub_mode.publish(m)


# ═══════════════════════════════════════════════
#  ROBOT KOL KONTROL NODE
#  active=False iken joy/send/recv callback'leri çalışmaz, UART kapalı.
#  activate() portu açar + startup_lock yapar.
#  deactivate() asimetrik rampla yumuşak durur, hold paketleri gönderir,
#               portu kapatır, state'i sıfırlar.
# ═══════════════════════════════════════════════

class RobotKolControlNode(Node):
    """
    /joy -> UART robot kol komutları.

    UART1 (arm-step):  Waist, Shoulder, Elbow, Wrist step motorları
    UART2 (arm-servo): Gripper servo motorları

    Güvenlik: startup lock, asimetrik accel/decel, joy watchdog, shutdown ramp.
    """

    def __init__(self, step_serial_port='/dev/ttyUSB2', servo_serial_port='/dev/ttyUSB4'):
        super().__init__('robot_kol_control_node')

        self.declare_parameter('step_serial_port',  step_serial_port)
        self.declare_parameter('servo_serial_port', servo_serial_port)
        self.declare_parameter('arm_baud_rate',     ARM_SERIAL_BAUD)
        self.declare_parameter('send_interval_ms',  ARM_SEND_INTERVAL_MS)
        self.declare_parameter('deadzone',          ARM_DEADZONE)
        self.declare_parameter('accel_step',        ACCEL_STEP)
        self.declare_parameter('decel_step',        DECEL_STEP)
        self.declare_parameter('joy_timeout_sec',   JOY_TIMEOUT_SEC)

        self.step_serial_port  = self.get_parameter('step_serial_port').value
        self.servo_serial_port = self.get_parameter('servo_serial_port').value
        self.arm_baud_rate     = self.get_parameter('arm_baud_rate').value
        self.send_ms           = self.get_parameter('send_interval_ms').value
        self.deadzone          = self.get_parameter('deadzone').value
        self.accel_step        = self.get_parameter('accel_step').value
        self.decel_step        = self.get_parameter('decel_step').value
        self.joy_timeout       = self.get_parameter('joy_timeout_sec').value

        # Eksen hızları
        self.waist_speed    = 0.0;  self.target_waist_speed    = 0.0
        self.shoulder_speed = 0.0;  self.target_shoulder_speed = 0.0
        self.elbow_speed    = 0.0;  self.target_elbow_speed    = 0.0
        self.wrist_speed    = 0.0;  self.target_wrist_speed    = 0.0

        # Servo
        self.gripper_rot_angle = GRIPPER_ROT_CENTER
        self.gripper_oc_angle  = GRIPPER_OPEN_ANGLE

        # Durum bayrakları
        self._joy_received          = False
        self._joy_count             = 0
        self._last_joy_time         = 0.0
        self._last_gripper_update   = time.time()
        self._last_log              = 0.0
        self._step_feedback_buf     = ""
        self._servo_feedback_buf    = ""
        self._shutdown_in_progress  = False
        self._startup_complete      = False
        self._joy_timeout_warned    = False
        self._speed_lock            = Lock()

        # UART nesneleri activate()'de açılır
        self.step_ser  = None
        self.servo_ser = None
        self.active    = False

        self.create_subscription(Joy, 'joy', self._joy_cb, 10)

        self.pub_status    = self.create_publisher(String, 'arm/status',         10)
        self.pub_waist     = self.create_publisher(Int16,  'arm/waist_speed',    10)
        self.pub_shoulder  = self.create_publisher(Int16,  'arm/shoulder_speed', 10)
        self.pub_elbow     = self.create_publisher(Int16,  'arm/elbow_speed',    10)
        self.pub_wrist     = self.create_publisher(Int16,  'arm/wrist_speed',    10)
        self.pub_grip_rot  = self.create_publisher(Int16,  'arm/gripper_rot',    10)
        self.pub_grip_oc   = self.create_publisher(Int16,  'arm/gripper_oc',     10)
        self.pub_telemetry = self.create_publisher(String, '/marmara/telemetry_json', 10)

        self.create_timer(self.send_ms / 1000.0, self._send_cb)
        self.create_timer(0.02, self._recv_cb)  # 50 Hz

        self.get_logger().info(
            f'ROBOT KOL NODE HAZIR (pasif) | '
            f'STEP:{self.step_serial_port} SERVO:{self.servo_serial_port}'
        )

    # ─────────────── UART ───────────────

    def _open_port(self, port_path):
        try:
            ser = serial.Serial()
            ser.port         = port_path
            ser.baudrate     = self.arm_baud_rate
            ser.bytesize     = 8
            ser.parity       = 'N'
            ser.stopbits     = 1
            ser.timeout      = 0.0
            ser.write_timeout = 0.2
            ser.dtr = False
            ser.rts = False
            ser.open()
            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass
            self.get_logger().info(f'SERIAL OK: {port_path}')
            time.sleep(1.8)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            return ser
        except Exception as e:
            self.get_logger().error(f'SERIAL HATA ({port_path}): {e}')
            return None

    def _open_step_serial(self):
        self.step_ser = self._open_port(self.step_serial_port)

    def _open_servo_serial(self):
        self.servo_ser = self._open_port(self.servo_serial_port)

    def _build_step_packet(self):
        w_val = int(self.waist_speed);    w_dir = 1 if w_val >= 0 else 0;  w_spd = min(WAIST_SPEED_MAX,    abs(w_val))
        s_val = int(self.shoulder_speed); s_dir = 1 if s_val >= 0 else 0;  s_spd = min(SHOULDER_SPEED_MAX, abs(s_val))
        e_val = int(self.elbow_speed);    e_dir = 1 if e_val >= 0 else 0;  e_spd = min(ELBOW_SPEED_MAX,    abs(e_val))
        r_val = int(self.wrist_speed);    r_dir = 1 if r_val >= 0 else 0;  r_spd = min(WRIST_SPEED_MAX,    abs(r_val))
        return f"[{w_dir},{w_spd},{s_dir},{s_spd},{e_dir},{e_spd},{r_dir},{r_spd}]\n".encode('ascii')

    def _build_servo_packet(self):
        gr = max(0, min(180, int(self.gripper_rot_angle)))
        go = max(0, min(180, int(self.gripper_oc_angle)))
        return f"[{gr},{go}]\n".encode('ascii')

    def _write_port(self, ser, pkt, label, reopen_fn):
        if ser and ser.is_open:
            try:
                ser.write(pkt)
                ser.flush()
                return ser
            except Exception as e:
                self.get_logger().warn(f'{label} write hata: {e}')
                try:
                    ser.close()
                except Exception:
                    pass
                reopen_fn()
        return ser

    def _uart_send(self):
        self._write_port(self.step_ser,  self._build_step_packet(),  'STEP',  self._open_step_serial)
        self._write_port(self.servo_ser, self._build_servo_packet(), 'SERVO', self._open_servo_serial)

    def _read_port(self, ser, buf, label):
        if ser and ser.is_open:
            try:
                while ser.in_waiting > 0:
                    c = ser.read(1).decode('ascii', errors='ignore')
                    if c == '\n':
                        line = buf.strip()
                        buf  = ""
                        if line:
                            self.get_logger().debug(f'{label} FB: {line}')
                    else:
                        buf = buf + c if len(buf) < 256 else ""
            except Exception:
                pass
        return buf

    def _uart_recv(self):
        self._step_feedback_buf  = self._read_port(self.step_ser,  self._step_feedback_buf,  'STEP')
        self._servo_feedback_buf = self._read_port(self.servo_ser, self._servo_feedback_buf, 'SERVO')

    # ─────────────── LIFECYCLE ───────────────

    def startup_lock(self):
        self.get_logger().info('>>> STARTUP LOCK: Motorlar kilitli pozisyona aliniyor...')
        with self._speed_lock:
            self.target_waist_speed    = 0.0
            self.target_shoulder_speed = 0.0
            self.target_elbow_speed    = 0.0
            self.target_wrist_speed    = 0.0
        self.waist_speed = self.shoulder_speed = self.elbow_speed = self.wrist_speed = 0.0
        for _ in range(STARTUP_HOLD_PACKETS):
            self._uart_send()
            time.sleep(0.05)
        self._startup_complete = True
        self.get_logger().info(">>> MOTOR KILIDI AKTIF - Kol güvenli hold state'te")

    def activate(self):
        if self.active:
            return
        self.get_logger().info('>>> ROBOT KOL AKTIVASYON BAŞLADI...')
        if SERIAL_OK:
            self._open_step_serial()
            self._open_servo_serial()
        self._last_joy_time      = time.time()
        self._joy_timeout_warned = False
        self._joy_received       = False
        self.startup_lock()
        self.active = True
        self.get_logger().info('>>> ROBOT KOL AKTIF')

    def deactivate(self):
        if not self.active:
            return
        self.get_logger().info('>>> ROBOT KOL DEAKTIVASYON: Yumuşak frenleme...')
        self._shutdown_in_progress = True
        self.active = False  # _send_cb ve _joy_cb erken döner

        with self._speed_lock:
            self.target_waist_speed = self.target_shoulder_speed = 0.0
            self.target_elbow_speed = self.target_wrist_speed    = 0.0

        max_iter   = int(SHUTDOWN_RAMP_MAX_SEC * 1000 / self.send_ms)
        start_time = time.time()
        for i in range(max_iter):
            with self._speed_lock:
                self.waist_speed    = self._apply_accel(self.waist_speed,    0.0)
                self.shoulder_speed = self._apply_accel(self.shoulder_speed, 0.0)
                self.elbow_speed    = self._apply_accel(self.elbow_speed,    0.0)
                self.wrist_speed    = self._apply_accel(self.wrist_speed,    0.0)
            self._uart_send()
            if (abs(self.waist_speed)    < 1.0 and abs(self.shoulder_speed) < 1.0 and
                    abs(self.elbow_speed) < 1.0 and abs(self.wrist_speed)    < 1.0):
                self.get_logger().info(
                    f'>>> Motorlar durdu ({time.time()-start_time:.2f}s, {i+1} adim)'
                )
                break
            time.sleep(self.send_ms / 1000.0)

        for _ in range(STARTUP_HOLD_PACKETS):
            self._uart_send()
            time.sleep(0.05)

        if self.step_ser and self.step_ser.is_open:
            self.step_ser.close()
        if self.servo_ser and self.servo_ser.is_open:
            self.servo_ser.close()
        self.step_ser  = None
        self.servo_ser = None

        # State sıfırla (sonraki activate için hazır)
        self._startup_complete     = False
        self._joy_received         = False
        self._joy_timeout_warned   = False
        self._shutdown_in_progress = False
        with self._speed_lock:
            self.waist_speed = self.shoulder_speed = self.elbow_speed = self.wrist_speed = 0.0

        self.get_logger().info('>>> ROBOT KOL PASIF (portlar kapali)')

    def shutdown(self):
        self.deactivate()

    # ─────────────── HELPERS ───────────────

    def _dz(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        s = 1.0 if v > 0 else -1.0
        return s * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _apply_accel(self, current, target):
        decelerating = (abs(target) < abs(current) or
                        (current > 0 and target < 0) or
                        (current < 0 and target > 0))
        step = self.decel_step if decelerating else self.accel_step
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + (step if diff > 0 else -step)

    # ─────────────── CALLBACKS ───────────────

    def _joy_cb(self, msg):
        if not self.active:
            return
        self._joy_count += 1
        self._last_joy_time = time.time()
        if not self._joy_received:
            self._joy_received = True
            self.get_logger().info(f'>>> ILK JOY ALINDI! axes={len(msg.axes)} btn={len(msg.buttons)}')
        self._joy_timeout_warned = False

        raw_waist    = msg.axes[ARM_AXIS_WAIST]    if ARM_AXIS_WAIST    < len(msg.axes) else 0.0
        raw_shoulder = msg.axes[ARM_AXIS_SHOULDER] if ARM_AXIS_SHOULDER < len(msg.axes) else 0.0
        raw_elbow    = msg.axes[ARM_AXIS_ELBOW]    if ARM_AXIS_ELBOW    < len(msg.axes) else 0.0
        raw_wrist    = msg.axes[ARM_AXIS_WRIST]    if ARM_AXIS_WRIST    < len(msg.axes) else 0.0
        raw_lt       = msg.axes[ARM_AXIS_LT]       if ARM_AXIS_LT       < len(msg.axes) else -1.0
        raw_rt       = msg.axes[ARM_AXIS_RT]       if ARM_AXIS_RT       < len(msg.axes) else -1.0
        btn_x        = msg.buttons[ARM_BTN_X]      if ARM_BTN_X         < len(msg.buttons) else 0
        btn_y        = msg.buttons[ARM_BTN_Y]      if ARM_BTN_Y         < len(msg.buttons) else 0

        with self._speed_lock:
            self.target_waist_speed    = self._dz(raw_waist)    * WAIST_SPEED_MAX    * SPEED_SCALE
            self.target_shoulder_speed = self._dz(raw_shoulder) * SHOULDER_SPEED_MAX * SPEED_SCALE
            self.target_elbow_speed    = self._dz(raw_elbow)    * ELBOW_SPEED_MAX    * SPEED_SCALE
            self.target_wrist_speed    = self._dz(raw_wrist)    * WRIST_SPEED_MAX    * SPEED_SCALE

        now = time.time()
        dt  = min(now - self._last_gripper_update, 0.1)
        self._last_gripper_update = now

        rt_val = max(0.0, (raw_rt + 1.0) / 2.0)
        lt_val = max(0.0, (raw_lt + 1.0) / 2.0)
        if rt_val > 0.1:
            self.gripper_rot_angle = min(180.0, self.gripper_rot_angle + GRIPPER_DEG_PER_SEC * rt_val * dt)
        elif lt_val > 0.1:
            self.gripper_rot_angle = max(0.0,   self.gripper_rot_angle - GRIPPER_DEG_PER_SEC * lt_val * dt)

        if btn_x:
            self.gripper_oc_angle = int(GRIPPER_CLOSED_ANGLE)
        elif btn_y:
            self.gripper_oc_angle = int(GRIPPER_OPEN_ANGLE)

    def _send_cb(self):
        if not self.active:
            return
        if self._shutdown_in_progress:
            return
        if not self._startup_complete:
            return

        # Watchdog
        if self._joy_received:
            since = time.time() - self._last_joy_time
            if since > self.joy_timeout:
                with self._speed_lock:
                    self.target_waist_speed = self.target_shoulder_speed = 0.0
                    self.target_elbow_speed = self.target_wrist_speed    = 0.0
                if not self._joy_timeout_warned:
                    self._joy_timeout_warned = True
                    self.get_logger().warn(
                        f'!!! JOY TIMEOUT ({since:.2f}s) - Yumusak frenleme aktif'
                    )

        with self._speed_lock:
            tw = self.target_waist_speed
            ts = self.target_shoulder_speed
            te = self.target_elbow_speed
            tr = self.target_wrist_speed
            self.waist_speed    = self._apply_accel(self.waist_speed,    tw)
            self.shoulder_speed = self._apply_accel(self.shoulder_speed, ts)
            self.elbow_speed    = self._apply_accel(self.elbow_speed,    te)
            self.wrist_speed    = self._apply_accel(self.wrist_speed,    tr)

        self._uart_send()

        m = Int16();  m.data = int(self.waist_speed);         self.pub_waist.publish(m)
        m = Int16();  m.data = int(self.shoulder_speed);      self.pub_shoulder.publish(m)
        m = Int16();  m.data = int(self.elbow_speed);         self.pub_elbow.publish(m)
        m = Int16();  m.data = int(self.wrist_speed);         self.pub_wrist.publish(m)
        m = Int16();  m.data = int(self.gripper_rot_angle);   self.pub_grip_rot.publish(m)
        m = Int16();  m.data = int(self.gripper_oc_angle);    self.pub_grip_oc.publish(m)

        m      = String()
        m.data = (f'W:{int(self.waist_speed):+4d} S:{int(self.shoulder_speed):+4d} '
                  f'E:{int(self.elbow_speed):+4d} R:{int(self.wrist_speed):+4d} '
                  f'GR:{int(self.gripper_rot_angle)} GO:{int(self.gripper_oc_angle)}')
        self.pub_status.publish(m)

        telem = {
            'timestamp':   time.time(),
            'type':        'robot_kol',
            'speed_scale': SPEED_SCALE,
            'joints': {
                'waist':    int(self.waist_speed),
                'shoulder': int(self.shoulder_speed),
                'elbow':    int(self.elbow_speed),
                'wrist':    int(self.wrist_speed),
            },
            'gripper': {
                'rotation':   int(self.gripper_rot_angle),
                'open_close': int(self.gripper_oc_angle),
            },
            'watchdog': {
                'joy_active':     self._joy_received and not self._joy_timeout_warned,
                'time_since_joy': time.time() - self._last_joy_time if self._joy_received else -1,
            },
        }
        telem_msg      = String()
        telem_msg.data = json.dumps(telem)
        self.pub_telemetry.publish(telem_msg)

        now = time.time()
        if (now - self._last_log) >= 0.3:
            self._last_log = now
            self.get_logger().info(
                f'W:{int(self.waist_speed):+4d} S:{int(self.shoulder_speed):+4d} '
                f'E:{int(self.elbow_speed):+4d} R:{int(self.wrist_speed):+4d} | '
                f'GripRot:{int(self.gripper_rot_angle)} GripOC:{int(self.gripper_oc_angle)}'
            )

    def _recv_cb(self):
        if not self.active:
            return
        self._uart_recv()


# ═══════════════════════════════════════════════
#  LAUNCH MANAGER (BİRLEŞİK - MUTEX)
# ═══════════════════════════════════════════════

class LaunchManager:
    """
    ZMQ REP üzerinden iki modu yönetir.
    Karşılıklı dışlayan (mutex): start manuel -> kol auto-stop, tersi de geçerli.

    Desteklenen launches: 'manuel_control', 'robot_kol'
    """

    def __init__(self, manuel_node, drill_node, arm_node):
        self.manuel_node = manuel_node
        self.drill_node  = drill_node
        self.arm_node    = arm_node

        self._manuel_active = False
        self._arm_active    = False
        self._op_lock       = Lock()  # eş zamanlı aktivasyon engeli

        self.ctx  = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.setsockopt(zmq.RCVTIMEO, 500)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(f"tcp://*:{LAUNCH_MANAGER_PORT}")
        self._running = True
        self._thread  = Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[LaunchManager] ZMQ REP tcp://*:{LAUNCH_MANAGER_PORT}")

    # ─────────────── MUTEX AKTIVASYON ───────────────

    def _activate_manuel(self):
        """Kol varsa durdur, rover'ı başlat."""
        if self._arm_active:
            print("[LaunchManager] Robot kol durduruluyor (mutex)...")
            self.arm_node.deactivate()
            self._arm_active = False
        self.manuel_node.activate()
        if self.drill_node:
            self.drill_node.activate()
        self._manuel_active = True
        print("[LaunchManager] >>> MANUEL KONTROL AKTIF")

    def _deactivate_manuel(self):
        """Rover ve drill'i durdur."""
        if not self._manuel_active:
            return
        if self.drill_node:
            self.drill_node.deactivate()
        self.manuel_node.deactivate()
        self._manuel_active = False
        print("[LaunchManager] >>> MANUEL KONTROL DURDURULDU")

    def _activate_arm(self):
        """Rover varsa durdur, kolu başlat."""
        if self._manuel_active:
            print("[LaunchManager] Manuel kontrol durduruluyor (mutex)...")
            if self.drill_node:
                self.drill_node.deactivate()
            self.manuel_node.deactivate()
            self._manuel_active = False
        self.arm_node.activate()
        self._arm_active = True
        print("[LaunchManager] >>> ROBOT KOL AKTIF")

    def _deactivate_arm(self):
        """Kolu yumuşak durdur."""
        if not self._arm_active:
            return
        self.arm_node.deactivate()
        self._arm_active = False
        print("[LaunchManager] >>> ROBOT KOL DURDURULDU")

    # ─────────────── LOOP ───────────────

    def _loop(self):
        while self._running:
            try:
                raw = self.sock.recv_string()
            except zmq.Again:
                continue
            except Exception:
                break

            try:
                cmd = json.loads(raw)
            except Exception:
                self.sock.send_string(json.dumps({'success': False, 'message': 'Gecersiz JSON'}))
                continue

            action = cmd.get('action', '')
            launch = cmd.get('launch', '')
            result = None

            with self._op_lock:
                if action == 'ping':
                    result = {
                        'success': True,
                        'message': 'pong',
                        'timestamp': time.time(),
                        'available_launches': ['manuel_control', 'robot_kol'],
                    }

                elif action == 'start' and launch == 'manuel_control':
                    if not self._manuel_active:
                        # PC timeout'u önlemek için yanıtı blocking işlemden önce gönder
                        self.sock.send_string(json.dumps(
                            {'success': True, 'message': 'manuel_control BASLATILIYOR', 'pid': 0}
                        ))
                        self._activate_manuel()
                        result = None  # Yanıt zaten gönderildi
                    else:
                        result = {'success': True, 'message': 'manuel_control zaten aktif', 'pid': 0}

                elif action == 'start' and launch == 'robot_kol':
                    if not self._arm_active:
                        # PC timeout'u önlemek için yanıtı blocking işlemden önce gönder
                        self.sock.send_string(json.dumps(
                            {'success': True, 'message': 'robot_kol BASLATILIYOR', 'pid': 0}
                        ))
                        self._activate_arm()
                        result = None  # Yanıt zaten gönderildi
                    else:
                        result = {'success': True, 'message': 'robot_kol zaten aktif', 'pid': 0}

                elif action == 'stop' and launch == 'manuel_control':
                    self._deactivate_manuel()
                    result = {'success': True, 'message': 'manuel_control durduruldu'}

                elif action == 'stop' and launch == 'robot_kol':
                    self._deactivate_arm()
                    result = {'success': True, 'message': 'robot_kol durduruldu'}

                elif action == 'stop_all':
                    self._deactivate_manuel()
                    self._deactivate_arm()
                    result = {'success': True, 'results': {}}

                elif action == 'status':
                    result = {
                        'success': True,
                        'launches': {
                            'manuel_control': {
                                'running':      self._manuel_active,
                                'pid':          0,
                                'description':  'Manuel Kontrol (BLDC + Step + Drill)',
                                'recent_logs':  [],
                            },
                            'robot_kol': {
                                'running':      self._arm_active,
                                'pid':          0,
                                'description':  'Robot Kol Kontrol (Step + Servo)',
                                'recent_logs':  [],
                            },
                        },
                    }

                else:
                    result = {'success': False, 'message': f'Bilinmeyen: {action}/{launch}'}

            if result is not None:
                self.sock.send_string(json.dumps(result))

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=2)
        self.sock.close()
        self.ctx.term()


# ═══════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Marmara Rover - Jetson All-in-One v3 (Birlesik: Rover + Robot Kol)'
    )
    # Rover UART
    parser.add_argument('--rover-bldc1', default='/dev/ttyUSB1',
                        help='UART1: BLDC anakart 1 (default: /dev/ttyUSB1)')
    parser.add_argument('--rover-bldc2', default='/dev/ttyUSB5',
                        help='UART2: BLDC anakart 2 (default: /dev/ttyUSB5)')
    parser.add_argument('--rover-step',  default='/dev/ttyUSB0',
                        help='UART3: Step motor Arduino (default: /dev/ttyUSB0)')
    # Drill UART
    parser.add_argument('--drill-port', default='/dev/ttyUSB3',
                        help='UART4: Drill mikrokontrolcu (default: /dev/ttyUSB3)')
    parser.add_argument('--drill-baud', type=int, default=DRILL_BAUD,
                        help=f'Drill baud rate (default: {DRILL_BAUD})')
    # Arm UART
    parser.add_argument('--arm-step',  default='/dev/ttyUSB2',
                        help='UART5: Robot kol step karti (default: /dev/ttyUSB2)')
    parser.add_argument('--arm-servo', default='/dev/ttyUSB4',
                        help='UART6: Robot kol servo karti (default: /dev/ttyUSB4)')
    # Özellik anahtarları
    parser.add_argument('--no-drill',          action='store_true', help='Drill devre disi')
    parser.add_argument('--no-logitech',       action='store_true', help='Logitech kamera devre disi')
    parser.add_argument('--no-realsense',      action='store_true', help='RealSense kamera devre disi')
    parser.add_argument('--no-aruco',          action='store_true', help='ArUco tespiti devre disi')
    parser.add_argument('--no-launch-manager', action='store_true',
                        help='LaunchManager olusturma')
    parser.add_argument('--cam-id',   type=int, default=0,          help='Logitech camera ID (default: 0)')
    parser.add_argument('--aruco-source', choices=['realsense', 'logitech'], default='realsense',
                        help='ArUco icin kamera kaynagi (default: realsense)')
    parser.add_argument('--aruco-dict', default='DICT_5X5_250',
                        help='ArUco sozluk adi (default: DICT_5X5_250)')
    args = parser.parse_args()

    print('=' * 72)
    print('  MARMARA ROVER - JETSON ALL-IN-ONE v3 (BİRLEŞİK: ROVER + ROBOT KOL)')
    print('=' * 72)
    print(f'  ROVER BLDC1  : {args.rover_bldc1}')
    print(f'  ROVER BLDC2  : {args.rover_bldc2}')
    print(f'  ROVER STEP   : {args.rover_step}')
    print(f'  DRILL        : {"KAPALI" if args.no_drill else f"AKTIF ({args.drill_port} @ {args.drill_baud})"}')
    print(f'  ARM STEP     : {args.arm_step}')
    print(f'  ARM SERVO    : {args.arm_servo}')
    print(f'  Logitech     : {"KAPALI" if args.no_logitech else f"AKTIF (cam:{args.cam_id})"}')
    print(f'  RealSense    : {"KAPALI" if args.no_realsense else "AKTIF"}')
    print(f'  ArUco        : {"KAPALI" if args.no_aruco else f"AKTIF (src:{args.aruco_source}, dict:{args.aruco_dict})"}')
    print(f'  ZMQ Joy      : tcp://*:{JOY_RECV_PORT}')
    print(f'  ZMQ Mgr      : tcp://*:{LAUNCH_MANAGER_PORT}')
    print('-' * 72)
    print('  MOD GECİSİ (ZMQ port 5560):')
    print('    {action:"start", launch:"manuel_control"} -> Rover modu aktif')
    print('    {action:"start", launch:"robot_kol"}      -> Kol modu aktif')
    print('    Karsilıklı dislayan: biri baslatilinca digeri otomatik durur.')
    print('-' * 72)
    print('  UDEV ONERISI (kalici port adlandirma):')
    print('    /etc/udev/rules.d/99-rover.rules icine:')
    print('    ATTRS{serial}=="<serial_no>", SYMLINK+="rover_bldc1"')
    print('    Sonra: sudo udevadm control --reload && sudo udevadm trigger')
    print('=' * 72)

    rclpy.init()
    nodes = []

    # ── Logitech ──
    logi_node = None
    if not args.no_logitech:
        try:
            logi_node = LogitechPublisherNode()
            nodes.append(logi_node)
            print("[OK] Logitech node baslatildi")
        except Exception as e:
            print(f"[UYARI] Logitech baslatilamadi: {e}")

    # ── RealSense ──
    rs_node = None
    if not args.no_realsense and REALSENSE_OK:
        try:
            rs_node = RealSensePublisherNode()
            nodes.append(rs_node)
            print("[OK] RealSense node baslatildi")
        except Exception as e:
            print(f"[UYARI] RealSense baslatilamadi: {e}")
    elif not args.no_realsense and not REALSENSE_OK:
        print("[UYARI] pyrealsense2 yuklenmemis - RealSense devre disi")

    # ── ArUco ──
    aruco_node = None
    if not args.no_aruco:
        src_topic = ('/realsense/rgb/image_raw' if args.aruco_source == 'realsense'
                     else '/logitech/image_raw')
        try:
            aruco_node = ArucoDetectionNode(image_topic=src_topic, dictionary=args.aruco_dict)
            nodes.append(aruco_node)
            print(f"[OK] ArUco node baslatildi (kaynak: {src_topic})")
        except Exception as e:
            print(f"[UYARI] ArUco baslatilamadi: {e}")

    # ── ZMQ Bridge ──
    bridge_node = ZMQBridgeNode()
    nodes.append(bridge_node)

    # ── Manuel Kontrol (pasif - activate() ile açılır) ──
    manuel_node = ManuelControlNode(
        serial_port=args.rover_bldc1,
        serial_port2=args.rover_bldc2,
        serial_port3=args.rover_step,
    )
    nodes.append(manuel_node)

    # ── Drill (pasif - activate() ile açılır) ──
    drill_node = None
    if not args.no_drill:
        try:
            drill_node = DrillControlNode(
                drill_port=args.drill_port,
                drill_baud=args.drill_baud,
            )
            nodes.append(drill_node)
            print(f"[OK] Drill node olusturuldu ({args.drill_port}) - pasif")
        except Exception as e:
            print(f"[UYARI] Drill node olusturulamadi: {e}")

    # ── Robot Kol (pasif - activate() ile açılır) ──
    arm_node = RobotKolControlNode(
        step_serial_port=args.arm_step,
        servo_serial_port=args.arm_servo,
    )
    nodes.append(arm_node)

    # ── Executor ──
    executor = MultiThreadedExecutor(num_threads=10)
    for n in nodes:
        executor.add_node(n)

    # ── Launch Manager ──
    launch_mgr = None
    if not args.no_launch_manager:
        launch_mgr = LaunchManager(
            manuel_node=manuel_node,
            drill_node=drill_node,
            arm_node=arm_node,
        )

    print(f'\n[HAZIR] {len(nodes)} node aktif (manuel + kol pasif, kamera/aruco aktif)')
    print('[HAZIR] PC arayuzunden "MANUEL BASLAT" veya "ROBOT KOL BASLAT" tiklayin\n')

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n[KAPATILIYOR] Guvenli kapaniş başliyor...')
    finally:
        # Aktif modu durdur
        if launch_mgr:
            launch_mgr._deactivate_manuel()
            launch_mgr._deactivate_arm()
        else:
            manuel_node.deactivate()
            arm_node.deactivate()
            if drill_node:
                drill_node.deactivate()

        # Altyapı kapat
        bridge_node.shutdown()
        if aruco_node:
            aruco_node.shutdown()
        if logi_node:
            logi_node.shutdown()
        if rs_node:
            rs_node.shutdown()
        if launch_mgr:
            launch_mgr.shutdown()

        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
        print('[KAPATILDI]')


if __name__ == '__main__':
    main()
