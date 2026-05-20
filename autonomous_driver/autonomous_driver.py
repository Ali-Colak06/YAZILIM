#!/usr/bin/env python3
"""
================================================================================
  MARMARA ROVER – Otonom Sürüş Kodu  v2
  ERC 2026 Uyumlu  |  ArUco Navigasyon + YOLO Engel Kaçınma
  4-Tekerlek Bağımsız Step Motor Sürücü
================================================================================

ERC 2026 Navigation Traverse Task (§7.3.2.1):
  - 4 waypoint, herhangi sırayla; 20 dakika yürütme penceresi
  - ArUco landmark: DICT_5X5_100, ID 51-64, 150 × 150 mm işaret tabelası
  - No GNSS + Full Autonomy = maks puan (%100 traverse + %100 autonomy)

Step motor mimarisi (4 bağımsız):
  m0 = Sol-Ön  (LF)   m1 = Sağ-Ön  (RF)
  m2 = Sol-Arka (LR)  m3 = Sağ-Arka (RR)
  Arduino formatı: "MOTOR:d0,d1,d2,d3\\n"  (d ∈ {'L','R','S'})

ZMQ portları:
  5557 → komut girişi (SUB)     5561 → adım PUB
  5562 → waypoint alımı (SUB)   6001 → telemetri (SUB)
  6000 → Annotated Logitech PUB   6002/6003 → Annotated RealSense PUB
  6004 → sağlık PUB
  (manuel modda Rover3.py, otonom modda autonomous_driver.py 6000/6002/6003'ü bind eder; mutex)

BLDC protokolü (Rover.py ile özdeş):
  struct '<HhhH'  →  start=0xABCD, steer(i16), speed(i16), checksum
  board 0 = Sol taraf BLDC    board 1 = Sağ taraf BLDC

Çalıştırma:
  python3 autonomous_driver.py --detect-pt yolo_det.pt --seg-pt yolo_seg.pt
  python3 autonomous_driver.py --no-yolo --fake-obstacle right   # geliştirme

Waypoint gönderimi (ground station → ZMQ 5562):
  {"cmd": "start_mission", "targets": [51, 52, 53, 54], "approach_dist": 1.5}
  {"cmd": "abort"}

Bağımlılıklar:
  pip3 install pyzmq pyserial opencv-python numpy ultralytics
================================================================================
"""

import argparse
import json
import logging
import math
import os
import re
import signal
import struct
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from threading import Event, Lock, Thread
from typing import Optional

import cv2
import numpy as np
import zmq

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

try:
    from ultralytics import YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False


# ═══════════════════════════════════════════════════════════════════════════
#  Logging
# ═══════════════════════════════════════════════════════════════════════════

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)-22s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('otonom')

if not SERIAL_OK:
    log.warning('pyserial yok → pip3 install pyserial')
if not YOLO_OK:
    log.warning('ultralytics yok → pip3 install ultralytics  (veya --no-yolo kullan)')


# ═══════════════════════════════════════════════════════════════════════════
#  1. SABİTLER & PROTOKOL
# ═══════════════════════════════════════════════════════════════════════════

# ── BLDC UART protokolü (Rover.py ile özdeş) ────────────────────────────
START_FRAME   = 0xABCD
COMMAND_FMT   = '<HhhH'          # start, steer(i16), speed(i16), checksum
FEEDBACK_FMT  = '<HhhhhhhHH'
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FMT)   # 18 bayt

# ── Hız sabitleri ────────────────────────────────────────────────────────
SPEED_FWD     = 100   # İleri sürüş BLDC hızı
SPEED_TURN    = 80    # Yerinde dönüş BLDC hızı (her taraf)

# ── ERC 2026 ArUco standardı (§7 Şekil 9-10) ────────────────────────────
ARUCO_DICT_ID      = cv2.aruco.DICT_5X5_100
ARUCO_VALID_IDS    = frozenset(range(0, 65))   # ID 51 .. 64
ARUCO_MARKER_LEN   = 0.150   # metre — 150 mm ± 2 mm

# Kamera iç parametreleri — RealSense D435i @ 848×480
CAMERA_MATRIX = np.array([
    [617.0,   0.0, 424.0],
    [  0.0, 617.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)  # fabrika kalibrasyonu yeterli

# ── Navigasyon PID (başlık hatası → diferansiyel BLDC hızı) ─────────────
NAV_PID_KP      = 80.0
NAV_PID_KI      = 0.0
NAV_PID_KD      = 20.0
NAV_PID_MAX_OUT = float(SPEED_FWD)   # Maks düzeltme = ileri hız

# ── Zamanlama sabitleri ──────────────────────────────────────────────────
SETTLE_MS            = 5        # Step → BLDC geçiş bekleme (ms)
TURN_HOLD_MS         = 500      # Dönüş süresi (ms)
WATCHDOG_TIMEOUT_S   = 10      # Veri kopukluğu E-stop eşiği
MISSION_TIMEOUT_S    = 20 * 60  # ERC §7.3.2.1: 20 dakika
SEARCH_TIMEOUT_S     = 120.0     # ArUco arama maks süresi
SEARCH_SCAN_STEP_S   = 2.0      # Tarama dönüş adımı
RETURN_HOME_TIMEOUT_S = 240.0   # Eve dönüş maks süresi
ESTOP_HOME_TIMEOUT_S  = 10.0    # E-stop home dönüş zaman aşımı
STEP_HOME_TOLERANCE   = 20      # Encoder toleransı (adım)
ARUCO_LOST_TICKS_MAX  = 25      # ~0.5s @ 50Hz — kayıp marker toleransı

# ── Navigasyon eşikleri ──────────────────────────────────────────────────
WAYPOINT_REACH_DIST_M = 1.5    # Bu mesafede waypoint ulaşıldı sayılır (m)
ALIGN_THRESHOLD_RAD   = 0.05   # ~3° — başlık hizalandı eşiği

# ── YOLO / Engel ────────────────────────────────────────────────────────
OBSTACLE_AREA_THRESH = 0.06
FRONT_CENTER_BAND    = (0.35, 0.65)
YOLO_CONF_THRESH     = 0.45

# ── ZMQ portları ────────────────────────────────────────────────────────
ZMQ_TELEMETRY_PORT     = 6001   # Sensors.py SUB
# Annotated kamera yayını — arc_electron'un dinlediği portlar.
# Manuel modda Rover3.py bu portları bind eder; otonom modda
# autonomous_driver.py YOLO+ArUco işaretli kareleri buralara basar.
# rover_launcher.py mutex'i sayesinde aynı anda iki tarafın binding
# yapması mümkün değildir.
ZMQ_OVERLAY_LOGI_PORT  = 6000
ZMQ_OVERLAY_RGB_PORT   = 6002
ZMQ_OVERLAY_DEPTH_PORT = 6003
ZMQ_HEALTH_PORT        = 6004
ZMQ_STEP_PORT          = 5561
ZMQ_WAYPOINT_PORT      = 5562
ZMQ_LAUNCH_PORT        = 5560

# ── FSM hızları ─────────────────────────────────────────────────────────
FSM_TICK_HZ  = 50
YOLO_HZ      = 10
ARUCO_HZ     = 10
OVERLAY_HZ   = 15
HEALTH_HZ    = 1

# ── Overlay renkleri ────────────────────────────────────────────────────
MASK_COLORS = [
    (0, 200, 0), (0, 0, 200), (200, 200, 0),
    (200, 0, 200), (0, 200, 200), (200, 100, 0),
]


# ═══════════════════════════════════════════════════════════════════════════
#  2. FROZEN DATACLASS'LAR
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class DriveCmd:
    """
    Immutable sürüş komutu.
    bldc_left / bldc_right: −1000..+1000 (SPEED_FWD = 100)
    motors: (LF, RF, LR, RR) her biri 'L' | 'R' | 'S'
    """
    bldc_left:  int
    bldc_right: int
    motors: tuple[str, str, str, str]


@dataclass(frozen=True)
class ArucoDetection:
    """Tek bir ArUco işaretçisinin pose tahmini."""
    marker_id:       int
    distance_m:      float   # kameraya olan mesafe (m)
    heading_err_rad: float   # atan2(tx, tz): + = sağda, − = solda
    tvec:            tuple[float, float, float]   # (x, y, z) metre


@dataclass(frozen=True)
class WaypointList:
    """Ground station'dan alınan görev listesi."""
    targets:       tuple[int, ...]   # Ziyaret edilecek ArUco ID'leri
    approach_dist_m: float = WAYPOINT_REACH_DIST_M
    return_home:   bool    = True


# Önceden tanımlı sürüş komutları
_DRIVE_STOP    = DriveCmd(0, 0, ('S', 'S', 'S', 'S'))
_DRIVE_FORWARD = DriveCmd(SPEED_FWD, SPEED_FWD, ('S', 'S', 'S', 'S'))
_DRIVE_TURN_L  = DriveCmd(-SPEED_TURN,  SPEED_TURN, ('L', 'L', 'L', 'L'))
_DRIVE_TURN_R  = DriveCmd( SPEED_TURN, -SPEED_TURN, ('R', 'R', 'R', 'R'))


# ═══════════════════════════════════════════════════════════════════════════
#  3. MOTOR BUS SOYUTLAMASI
# ═══════════════════════════════════════════════════════════════════════════

class MotorBus(ABC):
    """Motor komut soyutlaması — UART ya da CAN için ortak arayüz."""

    home_pos: Optional[list[int]] = None

    @abstractmethod
    def send_bldc(self, board: int, speed: int) -> None:
        """board: 0=Sol-BLDC, 1=Sağ-BLDC | speed: −1000..+1000"""

    @abstractmethod
    def send_step_motors(self, dirs: list[str]) -> None:
        """
        4 step motoru bağımsız yönetir.
        dirs: [d_LF, d_RF, d_LR, d_RR]  her biri 'L'|'R'|'S'
        Arduino: MOTOR:d0,d1,d2,d3\\n
        """

    def send_step(self, direction: str) -> None:
        """Tüm motorlara aynı yön (geriye dönük uyumluluk)."""
        d_map = {'LEFT': 'L', 'RIGHT': 'R', 'STOP': 'S',
                 'L': 'L', 'R': 'R', 'S': 'S'}
        d = d_map.get(direction.upper(), 'S')
        self.send_step_motors([d, d, d, d])

    def send_drive(self, cmd: DriveCmd) -> None:
        """DriveCmd'i BLDC + step motorlara atomik olarak uygular."""
        self.send_bldc(0, cmd.bldc_left)
        self.send_bldc(1, cmd.bldc_right)
        self.send_step_motors(list(cmd.motors))

    @abstractmethod
    def read_step_positions(self) -> Optional[list[int]]:
        """Mevcut 4 step encoder değeri. Yoksa None."""

    @abstractmethod
    def read_bldc_feedback(self) -> dict:
        """{'board1': {speedR,speedL,batV,temp}, 'board2': {...}}"""

    @abstractmethod
    def capture_home(self, timeout: float = 3.0) -> bool:
        """Açılışta encoder konumunu home olarak kaydet."""

    @abstractmethod
    def close(self) -> None:
        """Tüm portları kapat."""


# ── UART Gerçek Uygulama ─────────────────────────────────────────────────

class UARTMotorBus(MotorBus):
    """
    UART üzerinden BLDC + Step motor sürücüsü.
    BLDC paketi: struct '<HhhH'  (Rover.py ile özdeş)
    Step paketi: "MOTOR:L,R,S,L\\n"  (v2 — per-motor)
    """

    _log = logging.getLogger('otonom.uart')

    def __init__(self, port1: str, port2: str, step_port: str,
                 baud: int = 115200, step_baud: int = 115200):
        self.home_pos: Optional[list[int]] = None
        self._ser1 = self._ser2 = self._ser3 = None
        self._p1, self._p2, self._p3 = port1, port2, step_port
        self._baud, self._step_baud  = baud, step_baud

        self._fb1 = {'speedR': 0, 'speedL': 0, 'batV': 0.0, 'temp': 0.0}
        self._fb2 = {'speedR': 0, 'speedL': 0, 'batV': 0.0, 'temp': 0.0}
        self._fb_lock   = Lock()
        self._step_lock = Lock()
        self._step_pos  = [0, 0, 0, 0]
        self._step_pos_valid = False
        self._step_buf  = ''

        self._start_lo  = START_FRAME & 0xFF
        self._start_hi  = (START_FRAME >> 8) & 0xFF
        self._rx = [{'prev': None, 'collecting': False, 'buf': bytearray()}
                    for _ in range(2)]

        if not SERIAL_OK:
            self._log.warning('pyserial yok — motor komutları gönderilmeyecek')
            return

        self._open_ports()
        self._stop_evt = Event()
        Thread(target=self._read_bldc_loop, args=(0,),
               daemon=True, name='BLDCRead1').start()
        Thread(target=self._read_bldc_loop, args=(1,),
               daemon=True, name='BLDCRead2').start()
        Thread(target=self._read_step_loop,
               daemon=True, name='StepRead').start()

    def _open_ports(self):
        for port, attr, label, baud in [
            (self._p1, '_ser1', 'BLDC1', self._baud),
            (self._p2, '_ser2', 'BLDC2', self._baud),
            (self._p3, '_ser3', 'Step',  self._step_baud),
        ]:
            try:
                s = serial.Serial(port, baud,
                                  bytesize=8, parity='N', stopbits=1, timeout=0.0)
                setattr(self, attr, s)
                self._log.info('%s OK: %s', label, port)
            except Exception as exc:
                self._log.error('%s HATA %s: %s', label, port, exc)

    # ── Gönderim ────────────────────────────────────────────────────────

    def send_bldc(self, board: int, speed: int) -> None:
        speed = max(-1000, min(1000, int(speed)))
        steer = 0
        chk   = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
        pkt   = struct.pack(COMMAND_FMT, START_FRAME, steer, speed, chk)
        ser   = self._ser1 if board == 0 else self._ser2
        if ser and ser.is_open:
            try:
                ser.write(pkt)
            except Exception:
                self._reopen_bldc(board)

    def send_step_motors(self, dirs: list[str]) -> None:
        """
        4 motoru bağımsız sürücüsü.
        dirs: [LF, RF, LR, RR]  her biri 'L'|'R'|'S'
        Gönderim: "MOTOR:L,R,S,L\\n"
        """
        _map = {'L': 'L', 'R': 'R', 'S': 'S',
                'LEFT': 'L', 'RIGHT': 'R', 'STOP': 'S'}
        normalized = [_map.get(d.upper(), 'S') for d in dirs[:4]]
        # 4'ten az eleman geldiyse 'S' ile tamamla
        while len(normalized) < 4:
            normalized.append('S')
        cmd = 'MOTOR:' + ','.join(normalized) + '\n'
        if self._ser3 and self._ser3.is_open:
            try:
                self._ser3.write(cmd.encode('ascii'))
            except Exception:
                self._reopen_step()

    # ── Okuma ───────────────────────────────────────────────────────────

    def read_step_positions(self) -> Optional[list[int]]:
        with self._step_lock:
            return list(self._step_pos) if self._step_pos_valid else None

    def read_bldc_feedback(self) -> dict:
        with self._fb_lock:
            return {'board1': dict(self._fb1), 'board2': dict(self._fb2)}

    def capture_home(self, timeout: float = 3.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            pos = self.read_step_positions()
            if pos is not None:
                self.home_pos = pos
                self._log.info('Home position kaydedildi: %s', self.home_pos)
                return True
            time.sleep(0.05)
        self.home_pos = [0, 0, 0, 0]
        self._log.warning('Encoder yanıtı yok → home=[0,0,0,0] varsayıldı')
        return False

    def close(self) -> None:
        if hasattr(self, '_stop_evt'):
            self._stop_evt.set()
        for attr in ('_ser1', '_ser2', '_ser3'):
            s = getattr(self, attr, None)
            if s and s.is_open:
                s.close()

    # ── Arka plan okuma döngüleri ────────────────────────────────────────

    def _read_bldc_loop(self, board: int):
        r = self._rx[board]
        while not self._stop_evt.is_set():
            ser = self._ser1 if board == 0 else self._ser2
            if not ser or not ser.is_open:
                time.sleep(0.1)
                continue
            try:
                data = ser.read(256)
            except Exception:
                time.sleep(0.1)
                continue
            for b in data:
                if r['prev'] is None:
                    r['prev'] = b
                    continue
                if not r['collecting']:
                    if r['prev'] == self._start_lo and b == self._start_hi:
                        r['collecting'] = True
                        r['buf']        = bytearray([r['prev'], b])
                else:
                    r['buf'].append(b)
                    if len(r['buf']) == FEEDBACK_SIZE:
                        self._parse_feedback(bytes(r['buf']), board)
                        r['collecting'] = False
                        r['buf']        = bytearray()
                r['prev'] = b
            time.sleep(0.001)

    def _parse_feedback(self, raw: bytes, board: int):
        try:
            fields = struct.unpack(FEEDBACK_FMT, raw)
        except struct.error:
            return
        start, _, _, speedR, speedL, batV_raw, temp_raw, _, recv_csum = fields
        calc = (start ^ (fields[1] & 0xFFFF) ^ (fields[2] & 0xFFFF)
                ^ (fields[3] & 0xFFFF) ^ (fields[4] & 0xFFFF)
                ^ (fields[5] & 0xFFFF) ^ (fields[6] & 0xFFFF)
                ^ (fields[7] & 0xFFFF)) & 0xFFFF
        if start != START_FRAME or calc != recv_csum:
            return
        fb = {'speedR': speedR, 'speedL': speedL,
              'batV': batV_raw / 100.0, 'temp': temp_raw / 10.0}
        with self._fb_lock:
            if board == 0:
                self._fb1 = fb
            else:
                self._fb2 = fb

    def _read_step_loop(self):
        _POS_RE = re.compile(r'^POS:(-?\d+),(-?\d+),(-?\d+),(-?\d+)$')
        while not self._stop_evt.is_set():
            if not self._ser3 or not self._ser3.is_open:
                time.sleep(0.1)
                continue
            try:
                while self._ser3.in_waiting > 0:
                    c = self._ser3.read(1).decode('ascii', errors='ignore')
                    if c == '\n':
                        line = self._step_buf.strip()
                        self._step_buf = ''
                        m = _POS_RE.match(line)
                        if m:
                            vals = [int(g) for g in m.groups()]
                            with self._step_lock:
                                self._step_pos       = vals
                                self._step_pos_valid = True
                    else:
                        self._step_buf += c
            except Exception:
                pass
            time.sleep(0.01)

    def _reopen_bldc(self, board: int):
        attr = '_ser1' if board == 0 else '_ser2'
        port = self._p1 if board == 0 else self._p2
        try:
            s = serial.Serial(port, self._baud,
                              bytesize=8, parity='N', stopbits=1, timeout=0.0)
            setattr(self, attr, s)
        except Exception:
            setattr(self, attr, None)

    def _reopen_step(self):
        try:
            self._ser3 = serial.Serial(
                self._p3, self._step_baud,
                bytesize=8, parity='N', stopbits=1, timeout=0.0)
        except Exception:
            self._ser3 = None


# ── CAN Stub ─────────────────────────────────────────────────────────────

class CANMotorBus(MotorBus):
    """
    CAN otobüsüne geçiş için iskelet. Henüz uygulanmadı.

    Önerilen CAN ID şeması:
      0x100 / 0x101  BLDC komutu (board 0/1)  →  struct '<HhhH' (8 bayt)
      0x110          Step grup komutu          →  4 bayt: [d0,d1,d2,d3]
                                                   0=STOP, 1=LEFT, 2=RIGHT
      0x180–0x183   Encoder feedback (per motor)  →  '<i' signed 32-bit
      0x190–0x191   BLDC RPM feedback             →  '<hh' speedR speedL
    Sensors.py 0x200+ alanıyla çakışmaz.
    """

    def __init__(self):
        raise SystemExit(
            '[CANMotorBus] CAN desteği henüz uygulanmadı.\n'
            'Lütfen --bus=uart kullanın.')

    def send_bldc(self, board, speed):
        raise NotImplementedError

    def send_step_motors(self, dirs):
        raise NotImplementedError

    def read_step_positions(self):
        raise NotImplementedError

    def read_bldc_feedback(self):
        raise NotImplementedError

    def capture_home(self, timeout=3.0):
        raise NotImplementedError

    def close(self):
        pass


# ═══════════════════════════════════════════════════════════════════════════
#  4. TELEMETRİ ABONESİ
# ═══════════════════════════════════════════════════════════════════════════

class TelemetrySubscriber(Thread):
    """Sensors.py ZMQ PUB port 6001'e abone olur. Watchdog timestamp'i tutar."""

    _log = logging.getLogger('otonom.telem')

    def __init__(self, host: str = '127.0.0.1', port: int = ZMQ_TELEMETRY_PORT):
        super().__init__(daemon=True, name='TelemetrySubscriber')
        self.state:           dict  = {}
        self.last_update_ts: float  = 0.0
        self._lock = Lock()
        self._stop = Event()
        self._addr = f'tcp://{host}:{port}'

    def get(self) -> dict:
        with self._lock:
            return dict(self.state)

    def stop(self):
        self._stop.set()

    def run(self):
        ctx  = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.setsockopt(zmq.RCVHWM,   2)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.connect(self._addr)
        self._log.info('Sensors.py bağlandı: %s', self._addr)
        while not self._stop.is_set():
            try:
                raw = sock.recv_string()
            except zmq.Again:
                continue
            except Exception as exc:
                self._log.error('Hata: %s', exc)
                time.sleep(0.2)
                continue
            try:
                data = json.loads(raw)
                with self._lock:
                    self.state           = data
                    self.last_update_ts  = time.time()
            except Exception:
                pass
        sock.close()


# ═══════════════════════════════════════════════════════════════════════════
#  5. KAMERA MERKEZİ (ROS2 Node)
# ═══════════════════════════════════════════════════════════════════════════

class CameraHub(Node):
    """ROS2 kamera topic'lerine abone olur; cv2 karelerini thread-safe saklar."""

    TOPICS = {
        'logitech':        '/logitech/image_raw',
        'realsense_color': '/realsense/rgb/image_raw',
        'realsense_depth': '/realsense/depth/image_rect',
    }

    def __init__(self):
        super().__init__('camera_hub')
        self.bridge = CvBridge()
        self._lock   = Lock()
        self._frames: dict[str, tuple[np.ndarray, float]] = {}
        for key, topic in self.TOPICS.items():
            enc = 'passthrough' if 'depth' in key else 'bgr8'
            self.create_subscription(
                Image, topic,
                lambda msg, k=key, e=enc: self._cb(msg, k, e),
                5)
        self.get_logger().info('CameraHub başlatıldı')

    def _cb(self, msg: Image, key: str, encoding: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            with self._lock:
                self._frames[key] = (frame, time.time())
        except Exception as exc:
            self.get_logger().error('%s: %s', key, exc)

    def get_frame(self, key: str) -> Optional[tuple[np.ndarray, float]]:
        with self._lock:
            return self._frames.get(key)

    def latest(self) -> dict[str, tuple[np.ndarray, float]]:
        with self._lock:
            return dict(self._frames)


# ═══════════════════════════════════════════════════════════════════════════
#  6. YOLO ÇIKARSAMA WORKER
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class DetectionResult:
    boxes:   np.ndarray
    scores:  np.ndarray
    classes: np.ndarray


@dataclass
class SegmentationResult:
    boxes:   np.ndarray
    scores:  np.ndarray
    classes: np.ndarray
    masks:   list[np.ndarray] = field(default_factory=list)


_EMPTY_DET = DetectionResult(np.empty((0, 4)), np.empty(0), np.empty(0, dtype=int))
_EMPTY_SEG = SegmentationResult(np.empty((0, 4)), np.empty(0), np.empty(0, dtype=int), [])


class YoloWorker(Thread):
    """RealSense renk karesinde ~10Hz'de detection + segmentation çalıştırır."""

    _log = logging.getLogger('otonom.yolo')

    def __init__(self, cameras: CameraHub, detect_pt: Optional[str],
                 seg_pt: Optional[str], no_yolo: bool = False,
                 fake_obstacle: str = 'none', conf: float = YOLO_CONF_THRESH):
        super().__init__(daemon=True, name='YoloWorker')
        self._cameras     = cameras
        self._no_yolo     = no_yolo
        self._fake        = fake_obstacle
        self._conf        = conf
        self._stop        = Event()
        self._lock        = Lock()
        self._det: Optional[DetectionResult]      = None
        self._seg: Optional[SegmentationResult]   = None
        self._infer_count = 0

        self.det_model = None
        self.seg_model = None

        if no_yolo:
            self._log.info('--no-yolo: model yüklenmedi')
            return
        if not YOLO_OK:
            self._log.warning('ultralytics yüklü değil')
            return
        if detect_pt:
            self.det_model = YOLO(detect_pt)
            self._log.info('Detection model: %s', detect_pt)
        if seg_pt:
            self.seg_model = YOLO(seg_pt)
            self._log.info('Segmentation model: %s', seg_pt)

    def stop(self):
        self._stop.set()

    def get_results(self) -> tuple[Optional[DetectionResult], Optional[SegmentationResult]]:
        with self._lock:
            return self._det, self._seg

    def infer_count(self) -> int:
        return self._infer_count

    def run(self):
        interval = 1.0 / YOLO_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            frame_data = self._cameras.get_frame('realsense_color')
            if frame_data is not None:
                frame, _ = frame_data
                if self._fake != 'none':
                    self._inject_fake(frame)
                elif self._no_yolo or (self.det_model is None and self.seg_model is None):
                    with self._lock:
                        self._det = _EMPTY_DET
                        self._seg = _EMPTY_SEG
                else:
                    self._run_inference(frame)
            elapsed = time.monotonic() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    def _run_inference(self, frame: np.ndarray):
        det = seg = None
        if self.det_model:
            try:
                res = self.det_model.predict(frame, verbose=False, conf=self._conf)
                if res and res[0].boxes is not None and len(res[0].boxes):
                    b = res[0].boxes
                    det = DetectionResult(
                        boxes   = b.xyxy.cpu().numpy(),
                        scores  = b.conf.cpu().numpy(),
                        classes = b.cls.cpu().numpy().astype(int),
                    )
                else:
                    det = _EMPTY_DET
            except Exception as exc:
                self._log.error('Detection hatası: %s', exc)
                det = _EMPTY_DET

        if self.seg_model:
            try:
                res = self.seg_model.predict(frame, verbose=False, conf=self._conf)
                if res and res[0].boxes is not None and len(res[0].boxes):
                    b = res[0].boxes
                    masks = []
                    if res[0].masks is not None:
                        h, w = frame.shape[:2]
                        for m in res[0].masks.data:
                            mf = m.cpu().numpy()
                            mf = cv2.resize(mf, (w, h))
                            masks.append((mf > 0.5).astype(np.uint8))
                    seg = SegmentationResult(
                        boxes   = b.xyxy.cpu().numpy(),
                        scores  = b.conf.cpu().numpy(),
                        classes = b.cls.cpu().numpy().astype(int),
                        masks   = masks,
                    )
                else:
                    seg = _EMPTY_SEG
            except Exception as exc:
                self._log.error('Segmentation hatası: %s', exc)
                seg = _EMPTY_SEG

        with self._lock:
            if det is not None:
                self._det = det
            if seg is not None:
                self._seg = seg
            self._infer_count += 1

    def _inject_fake(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        if self._fake == 'left':
            box = np.array([[10, h * 0.2, w * 0.43, h * 0.8]])
        else:
            box = np.array([[w * 0.57, h * 0.2, w - 10, h * 0.8]])
        with self._lock:
            self._det = DetectionResult(box, np.array([0.92]), np.array([0]))
            self._seg = SegmentationResult(box, np.array([0.92]), np.array([0]), [])
            self._infer_count += 1


# ═══════════════════════════════════════════════════════════════════════════
#  7. ARUCO WORKER — ERC 2026 Standardı
# ═══════════════════════════════════════════════════════════════════════════

class ArucoWorker(Thread):
    """
    10Hz'de RealSense renk karesinde ERC 2026 ArUco işaretçilerini tespit eder.
    DICT_5X5_100 | ID 51–64 | 150 mm × 150 mm  (§7 Şekil 9-10)
    solvePnP ile 3D pose tahmini.
    """

    _log = logging.getLogger('otonom.aruco')

    # ERC 2026 ArUco işaretçi köşeleri (z=0 düzlemi, metre)
    _OBJ_PTS = np.array([
        [-ARUCO_MARKER_LEN / 2,  ARUCO_MARKER_LEN / 2, 0.0],
        [ ARUCO_MARKER_LEN / 2,  ARUCO_MARKER_LEN / 2, 0.0],
        [ ARUCO_MARKER_LEN / 2, -ARUCO_MARKER_LEN / 2, 0.0],
        [-ARUCO_MARKER_LEN / 2, -ARUCO_MARKER_LEN / 2, 0.0],
    ], dtype=np.float32)

    def __init__(self, cameras: CameraHub):
        super().__init__(daemon=True, name='ArucoWorker')
        self._cameras    = cameras
        self._stop       = Event()
        self._lock       = Lock()
        self._detections: dict[int, ArucoDetection] = {}
        self._raw_corners: list = []   # overlay için

        _dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        _params = cv2.aruco.DetectorParameters()

        # OpenCV 4.7+ yeni API; eski sürüm fallback
        try:
            self._detector = cv2.aruco.ArucoDetector(_dict, _params)
            self._new_api  = True
            self._log.info('ArucoDetector (yeni API) hazır — DICT_5X5_100')
        except AttributeError:
            self._dict     = _dict
            self._params   = _params
            self._new_api  = False
            self._log.info('ArucoDetector (eski API) hazır — DICT_5X5_100')

    def stop(self) -> None:
        self._stop.set()

    def get_detections(self) -> dict[int, ArucoDetection]:
        with self._lock:
            return dict(self._detections)

    def get_raw_corners(self) -> list:
        with self._lock:
            return list(self._raw_corners)

    def run(self):
        interval = 1.0 / ARUCO_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            frame_data = self._cameras.get_frame('realsense_color')
            if frame_data is not None:
                frame, _ = frame_data
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self._process(gray)
            elapsed = time.monotonic() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    def _process(self, gray: np.ndarray):
        if self._new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params)

        dets: dict[int, ArucoDetection] = {}
        raw: list = []

        if ids is not None and len(ids) > 0:
            for corner, mid in zip(corners, ids.flatten()):
                if int(mid) not in ARUCO_VALID_IDS:
                    continue
                img_pts = corner.reshape(4, 2).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(
                    self._OBJ_PTS, img_pts,
                    CAMERA_MATRIX, DIST_COEFFS,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                tx = float(tvec[0])
                ty = float(tvec[1])
                tz = float(tvec[2])
                dist        = math.sqrt(tx * tx + ty * ty + tz * tz)
                heading_err = math.atan2(tx, tz)   # + = sağ, − = sol
                dets[int(mid)] = ArucoDetection(
                    marker_id       = int(mid),
                    distance_m      = dist,
                    heading_err_rad = heading_err,
                    tvec            = (tx, ty, tz),
                )
                raw.append(corner)
                self._log.debug('ID=%d d=%.2fm h=%.3frad', int(mid), dist, heading_err)

        with self._lock:
            self._detections = dets
            self._raw_corners = raw


# ═══════════════════════════════════════════════════════════════════════════
#  8. PID DENETLEYİCİ
# ═══════════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Basit PID denetleyici.
    Navigasyon başlık hatası → diferansiyel BLDC hız çıktısı.
    Varsayılan: kp=80, ki=0, kd=20, limit=SPEED_FWD
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = NAV_PID_MAX_OUT):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._limit    = output_limit
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_ts:  Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_ts  = None

    def update(self, error: float) -> float:
        now = time.monotonic()
        dt  = (now - self._prev_ts) if self._prev_ts is not None else 0.02
        self._prev_ts = now

        self._integral += error * dt
        d_term          = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err  = error

        raw = self.kp * error + self.ki * self._integral + self.kd * d_term
        return max(-self._limit, min(self._limit, raw))


# ═══════════════════════════════════════════════════════════════════════════
#  9. KAÇINMA PLANLAYICISI (düzeltilmiş)
# ═══════════════════════════════════════════════════════════════════════════

class Action(Enum):
    DRIVE      = auto()
    TURN_LEFT  = auto()
    TURN_RIGHT = auto()


class AvoidancePlanner:
    """
    Engel tespitine göre DRIVE / TURN_LEFT / TURN_RIGHT kararı üretir.

    OBSTACLE_CLASS_IDS: Boş = tüm tespitler engel.
    Doluysa yalnızca listedeki sınıf ID'leri engel sayılır.

    NOT: v1'deki DRIVABLE_CLASS_IDS ismi ve mantığı hatalıydı — düzeltildi.
    """

    OBSTACLE_CLASS_IDS: list[int] = []   # Boş → tüm tespitler engel

    def decide(self,
               det:   Optional[DetectionResult],
               seg:   Optional[SegmentationResult],
               depth: Optional[np.ndarray]) -> Action:

        if det is None or len(det.boxes) == 0:
            return Action.DRIVE

        if depth is not None:
            h_f, w_f = depth.shape[:2]
        else:
            w_f, h_f = 848, 480

        front = []
        for box, cls_id in zip(det.boxes, det.classes):
            # Boş liste: tümü engel; dolu liste: sadece listede olanlar engel
            if self.OBSTACLE_CLASS_IDS and int(cls_id) not in self.OBSTACLE_CLASS_IDS:
                continue
            x1, y1, x2, y2 = box
            cx_n   = ((x1 + x2) / 2.0) / w_f
            area_n = (x2 - x1) * (y2 - y1) / (w_f * h_f)
            if (FRONT_CENTER_BAND[0] <= cx_n <= FRONT_CENTER_BAND[1]
                    and area_n >= OBSTACLE_AREA_THRESH):
                front.append(box)

        if not front:
            return Action.DRIVE

        if seg and seg.masks:
            obstacle_union = np.zeros((h_f, w_f), dtype=np.uint8)
            for mask in seg.masks:
                if mask.shape == obstacle_union.shape:
                    obstacle_union = np.maximum(obstacle_union, mask)
            drivable    = 1 - obstacle_union
            left_score  = float(np.mean(drivable[:, :w_f // 2]))
            right_score = float(np.mean(drivable[:, w_f // 2:]))
            return Action.TURN_RIGHT if right_score >= left_score else Action.TURN_LEFT

        if depth is not None:
            left_d  = float(np.mean(depth[:, :w_f // 2]))
            right_d = float(np.mean(depth[:, w_f // 2:]))
            return Action.TURN_RIGHT if right_d >= left_d else Action.TURN_LEFT

        avg_cx = float(np.mean([(b[0] + b[2]) / 2.0 for b in front]))
        return Action.TURN_LEFT if avg_cx > w_f / 2 else Action.TURN_RIGHT


# ═══════════════════════════════════════════════════════════════════════════
#  10. OVERLAY YAYINLAYıCı (düzeltilmiş)
# ═══════════════════════════════════════════════════════════════════════════

class OverlayPublisher(Thread):
    """
    ~15Hz'de kamera karelerini YOLO+ArUco sonuçlarıyla birleştirip ZMQ yayınlar.
    1Hz'de sağlık mesajı gönderir (port 6004).
    """

    _log = logging.getLogger('otonom.overlay')

    def __init__(self, cameras: CameraHub, yolo: YoloWorker,
                 aruco: ArucoWorker, telemetry: TelemetrySubscriber,
                 jpeg_q: int = 80):
        super().__init__(daemon=True, name='OverlayPublisher')
        self._cameras  = cameras
        self._yolo     = yolo
        self._aruco    = aruco
        self._telem    = telemetry
        self._jpeg_q   = jpeg_q
        self._stop     = Event()
        self._fsm      = None       # set_fsm() ile atanır
        self._counts   = {k: 0 for k in ('logitech', 'rs_rgb', 'rs_depth')}

        self._pub_logi   = None
        self._pub_rgb    = None
        self._pub_depth  = None
        self._pub_health = None
        self._pub_step   = None

    def set_step_pub(self, sock: zmq.Socket):
        self._pub_step = sock

    def set_fsm(self, fsm: object):
        """Sağlık durumu için FSM referansı."""
        self._fsm = fsm

    def stop(self):
        self._stop.set()

    def run(self):
        ctx = zmq.Context.instance()
        for attr, port, hwm in [
            ('_pub_logi',   ZMQ_OVERLAY_LOGI_PORT,  2),
            ('_pub_rgb',    ZMQ_OVERLAY_RGB_PORT,   2),
            ('_pub_depth',  ZMQ_OVERLAY_DEPTH_PORT, 2),
            ('_pub_health', ZMQ_HEALTH_PORT,         4),
        ]:
            s = ctx.socket(zmq.PUB)
            s.setsockopt(zmq.SNDHWM, hwm)
            try:
                s.bind(f'tcp://*:{port}')
                setattr(self, attr, s)
                self._log.info('ZMQ PUB :%d', port)
            except zmq.ZMQError as exc:
                self._log.warning('bind :%d → %s', port, exc)

        interval     = 1.0 / OVERLAY_HZ
        health_every = int(OVERLAY_HZ / HEALTH_HZ)
        tick         = 0

        while not self._stop.is_set():
            t0         = time.monotonic()
            det, seg   = self._yolo.get_results()
            aruco_raw  = self._aruco.get_raw_corners()
            frame_data = self._cameras.latest()

            logi_data = frame_data.get('logitech')
            if logi_data and self._pub_logi:
                out = self._draw_overlay(logi_data[0], det, seg, aruco_raw)
                self._send_jpg(self._pub_logi, out)
                self._counts['logitech'] += 1

            rgb_data = frame_data.get('realsense_color')
            if rgb_data and self._pub_rgb:
                out = self._draw_overlay(rgb_data[0], det, seg, aruco_raw)
                self._send_jpg(self._pub_rgb, out)
                self._counts['rs_rgb'] += 1

            depth_data = frame_data.get('realsense_depth')
            if depth_data and self._pub_depth:
                dn = cv2.normalize(depth_data[0], None, 0, 255,
                                   cv2.NORM_MINMAX, cv2.CV_8U)
                cm = cv2.applyColorMap(dn, cv2.COLORMAP_JET)
                self._send_jpg(self._pub_depth, cm)
                self._counts['rs_depth'] += 1

            if tick % health_every == 0 and self._pub_health:
                self._send_health()

            tick += 1
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _draw_overlay(self, frame: np.ndarray,
                      det: Optional[DetectionResult],
                      seg: Optional[SegmentationResult],
                      aruco_corners: list) -> np.ndarray:
        out = frame.copy()

        # Segmentation maskeleri
        if seg and seg.masks:
            overlay = out.copy()
            for i, mask in enumerate(seg.masks):
                if mask.shape[:2] == out.shape[:2]:
                    overlay[mask > 0] = MASK_COLORS[i % len(MASK_COLORS)]
            out = cv2.addWeighted(out, 0.65, overlay, 0.35, 0)

        # YOLO bbox'ları
        if det and len(det.boxes) > 0:
            for box, score, cls_id in zip(det.boxes, det.scores, det.classes):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 60, 255), 2)
                cv2.putText(out, f'C{int(cls_id)}:{score:.2f}',
                            (x1, max(0, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 60, 255), 1,
                            cv2.LINE_AA)

        # ArUco işaretçileri
        for corner in aruco_corners:
            pts = corner.reshape(4, 2).astype(np.int32)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)

        return out

    def _send_jpg(self, sock: zmq.Socket, frame: np.ndarray):
        try:
            h, w = frame.shape[:2]
            _, jpg = cv2.imencode('.jpg', frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
            meta = json.dumps({'ts': time.time(), 'w': w, 'h': h}).encode()
            sock.send_multipart([meta, jpg.tobytes()])
        except Exception:
            pass

    def _send_health(self):
        now   = time.time()
        frames = self._cameras.latest()
        telem_alive = (now - self._telem.last_update_ts) < WATCHDOG_TIMEOUT_S

        # Gerçek FSM durumuna göre sağlık
        fsm_state = 'UNKNOWN'
        if self._fsm is not None:
            try:
                fsm_state = self._fsm.state.name
            except AttributeError:
                pass
        _err_states = {'E_STOP', 'WATCHDOG_TRIGGERED'}
        status = 'error' if fsm_state in _err_states else 'healthy'

        streams = {
            'logitech':       {'alive': 'logitech'        in frames, 'count': self._counts['logitech']},
            'realsense_rgb':  {'alive': 'realsense_color' in frames, 'count': self._counts['rs_rgb']},
            'realsense_depth':{'alive': 'realsense_depth' in frames, 'count': self._counts['rs_depth']},
            'autonomous':     {'alive': True,                         'count': self._yolo.infer_count()},
            'telemetry':      {'alive': telem_alive,                  'count': 0},
        }
        try:
            self._pub_health.send_string(json.dumps({
                'timestamp': now,
                'status':    status,
                'fsm_state': fsm_state,
                'streams':   streams,
            }))
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
#  11. WAYPOINT ALICI
# ═══════════════════════════════════════════════════════════════════════════

class WaypointReceiver(Thread):
    """
    ZMQ SUB port 5562'den ground station waypoint mesajı alır.

    Beklenen format:
      {"cmd": "start_mission", "targets": [51, 52, 53, 54], "approach_dist": 1.5}
      {"cmd": "abort"}
    """

    _log = logging.getLogger('otonom.waypoints')

    def __init__(self, host: str = '127.0.0.1', port: int = ZMQ_WAYPOINT_PORT):
        super().__init__(daemon=True, name='WaypointReceiver')
        self._addr     = f'tcp://{host}:{port}'
        self._stop     = Event()
        self._lock     = Lock()
        self._pending: Optional[WaypointList] = None
        self._abort    = False

    def stop(self) -> None:
        self._stop.set()

    def pop_mission(self) -> Optional[WaypointList]:
        with self._lock:
            m = self._pending
            self._pending = None
            return m

    def pop_abort(self) -> bool:
        with self._lock:
            a = self._abort
            self._abort = False
            return a

    def run(self):
        ctx  = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.setsockopt(zmq.RCVHWM,   4)
        sock.setsockopt(zmq.RCVTIMEO, 500)
        sock.connect(self._addr)
        self._log.info('ZMQ SUB :%d  (waypoint alımı)', ZMQ_WAYPOINT_PORT)

        while not self._stop.is_set():
            try:
                raw = sock.recv_string()
            except zmq.Again:
                continue
            except Exception as exc:
                self._log.error('Hata: %s', exc)
                time.sleep(0.5)
                continue
            try:
                data = json.loads(raw)
                cmd  = data.get('cmd', '')
                if cmd == 'start_mission':
                    targets  = tuple(int(t) for t in data.get('targets', []))
                    approach = float(data.get('approach_dist', WAYPOINT_REACH_DIST_M))
                    ret_home = bool(data.get('return_home', True))
                    wpl = WaypointList(targets=targets,
                                       approach_dist_m=approach,
                                       return_home=ret_home)
                    with self._lock:
                        self._pending = wpl
                    self._log.info('Görev alındı: hedefler=%s', targets)
                elif cmd == 'abort':
                    with self._lock:
                        self._abort = True
                    self._log.warning('Abort komutu alındı')
            except (json.JSONDecodeError, KeyError, ValueError) as exc:
                self._log.warning('Geçersiz mesaj: %s', exc)

        sock.close()


# ═══════════════════════════════════════════════════════════════════════════
#  12. E-STOP (düzeltilmiş — per-motor home dönüşü)
# ═══════════════════════════════════════════════════════════════════════════

def execute_estop(bus: MotorBus):
    """
    Güvenli durdurma sekansı:
      1. BLDC hız → 0  (50ms arayla iki kez)
      2. Her step motoru bağımsız olarak kendi home pozisyonuna döndür
         (C-2 düzeltmesi: artık avg değil per-motor yön)
    """
    log.warning('━━━ E-STOP BAŞLATILIYOR ━━━')

    for _ in range(2):
        bus.send_bldc(0, 0)
        bus.send_bldc(1, 0)
        time.sleep(0.05)
    log.info('E-STOP: BLDC hız = 0')

    home = bus.home_pos
    if home is None:
        log.warning('E-STOP: Home bilgisi yok → STOP komutu')
        bus.send_step_motors(['S', 'S', 'S', 'S'])
        time.sleep(0.5)
        log.warning('━━━ E-STOP TAMAMLANDI ━━━')
        return

    log.info('E-STOP: Home hedef=%s  → kapalı çevrim dönüş', home)
    t_start = time.time()
    while time.time() - t_start < ESTOP_HOME_TIMEOUT_S:
        cur = bus.read_step_positions()
        if cur is None:
            log.warning('E-STOP: Encoder yanıt vermedi → zaman bazlı fallback')
            bus.send_step_motors(['S', 'S', 'S', 'S'])
            time.sleep(0.5)
            break

        diffs = [cur[i] - home[i] for i in range(min(len(cur), len(home)))]
        if all(abs(d) <= STEP_HOME_TOLERANCE for d in diffs):
            log.info('E-STOP: Home toleransına girildi (%s)', cur)
            break

        # C-2 DÜZELTME: her motor için bağımsız yön
        dirs = []
        for d in diffs:
            if d > STEP_HOME_TOLERANCE:
                dirs.append('L')
            elif d < -STEP_HOME_TOLERANCE:
                dirs.append('R')
            else:
                dirs.append('S')
        while len(dirs) < 4:
            dirs.append('S')
        bus.send_step_motors(dirs[:4])
        time.sleep(0.02)
    else:
        log.warning('E-STOP: UYARI — Home dönüşü zaman aşıldı!')

    bus.send_step_motors(['S', 'S', 'S', 'S'])
    log.warning('━━━ E-STOP TAMAMLANDI ━━━')


# ═══════════════════════════════════════════════════════════════════════════
#  13. WATCHDOG (düzeltilmiş — kamera başlangıç zaman damgası)
# ═══════════════════════════════════════════════════════════════════════════

class Watchdog(Thread):
    """
    Haberleşme kopukluğu koruması.
    Layer A — Sensors.py telemetrisi > 2s gelmediyse E-stop.
    Layer B — RealSense color karesi  > 2s gelmediyse E-stop.
              C-1 düzeltmesi: kamera hiç kare vermediyse de tetiklenir
              (başlangıçtan itibaren WATCHDOG_TIMEOUT_S geçti ise).
    """

    _log = logging.getLogger('otonom.watchdog')

    def __init__(self, fsm: object, telemetry: TelemetrySubscriber,
                 cameras: CameraHub):
        super().__init__(daemon=True, name='Watchdog')
        self._fsm         = fsm
        self._telem       = telemetry
        self._cameras     = cameras
        self._stop        = Event()
        # C-1: Watchdog'un başladığı zaman damgası
        self._camera_start_ts = time.time()

    def stop(self):
        self._stop.set()

    def run(self):
        self._log.info('Başlatıldı (timeout=%.1fs)', WATCHDOG_TIMEOUT_S)
        while not self._stop.is_set():
            if self._fsm.done:
                break
            now = time.time()

            # Layer A — Telemetri
            if (self._telem.last_update_ts > 0.0
                    and (now - self._telem.last_update_ts) > WATCHDOG_TIMEOUT_S):
                self._log.warning('Telemetri koptu → E-stop tetikleniyor')
                self._fsm.trigger_watchdog()
                break

            # Layer B — Kamera (C-1 düzeltmesi)
            rs_data = self._cameras.get_frame('realsense_color')
            if rs_data is None:
                # Kamera henüz hiç kare vermedi; başlangıçtan beri timeout geçti mi?
                if (now - self._camera_start_ts) > WATCHDOG_TIMEOUT_S:
                    self._log.warning('RealSense hiç kare vermedi → E-stop')
                    self._fsm.trigger_watchdog()
                    break
            else:
                _, ts = rs_data
                if (now - ts) > WATCHDOG_TIMEOUT_S:
                    self._log.warning('RealSense karesi stale → E-stop')
                    self._fsm.trigger_watchdog()
                    break

            time.sleep(0.2)   # 5Hz kontrol


# ═══════════════════════════════════════════════════════════════════════════
#  14. OTONOM FSM
# ═══════════════════════════════════════════════════════════════════════════

class State(Enum):
    # ── Engel kaçınma ──────────────────────────────────────────────────
    IDLE               = auto()
    SCAN               = auto()
    TURN_PREP          = auto()
    TURN               = auto()
    SETTLE_5MS         = auto()
    DRIVE              = auto()
    CHECK_OBSTACLE     = auto()
    # ── ERC 2026 Navigasyon (§7.3.2.1) ────────────────────────────────
    NAVIGATE_INIT      = auto()   # Sıradaki waypoint'e başla
    SEARCH_LANDMARK    = auto()   # Hedef ArUco'yu tara
    ALIGN_HEADING      = auto()   # PID ile başlık hizala
    APPROACH_WAYPOINT  = auto()   # Waypoint'e yaklaş
    WAYPOINT_REACHED   = auto()   # Waypoint tamamlandı
    RETURN_HOME        = auto()   # Eve dön
    MISSION_COMPLETE   = auto()   # Görev bitti
    # ── Terminal durumlar ──────────────────────────────────────────────
    E_STOP             = auto()
    WATCHDOG_TRIGGERED = auto()


class AutonomousFSM:
    """
    50Hz tick döngüsüyle çalışan sonlu durum makinesi.

    Sürüş kuralı (donanım gereksinimi):
      Step ve BLDC ASLA aynı anda komut almaz.
      TURN_PREP → TURN (step)  →  SETTLE_5MS (≥5ms)  →  [sonraki durum (BLDC)]
    """

    _log = logging.getLogger('otonom.fsm')

    def __init__(self, bus: MotorBus, cameras: CameraHub,
                 yolo: YoloWorker, aruco: ArucoWorker,
                 planner: AvoidancePlanner, waypoint_rcvr: WaypointReceiver,
                 step_zmq: zmq.Socket):
        self.bus           = bus
        self.cameras       = cameras
        self.yolo          = yolo
        self.aruco         = aruco
        self.planner       = planner
        self.waypoint_rcvr = waypoint_rcvr
        self._step_zmq     = step_zmq

        self.state         = State.IDLE
        self.done          = False
        self.estop_flag    = False
        self.watchdog_flag = False

        # Dönüş durumu
        self._turn_dir:    Optional[Action] = None
        self._t_turn_start = 0.0
        self._t_settle     = 0.0
        self._post_settle_state = State.SCAN   # Settle sonrası gidilecek durum

        # Navigasyon durumu
        self._waypoint_targets:    list[int] = []
        self._approach_dist:       float = WAYPOINT_REACH_DIST_M
        self._mission_return_home: bool  = True
        self._current_target:      Optional[int] = None
        self._mission_start_ts:    Optional[float] = None
        self._search_start_ts:     float = 0.0
        self._last_scan_turn_ts:   float = 0.0
        self._scan_turn_dir:       Action = Action.TURN_RIGHT
        self._aruco_lost_ticks:    int = 0
        self._return_home_start_ts: Optional[float] = None

        self._pid = PIDController(kp=NAV_PID_KP, ki=NAV_PID_KI,
                                  kd=NAV_PID_KD, output_limit=NAV_PID_MAX_OUT)

    # ── Dış tetikleyiciler ───────────────────────────────────────────────

    def trigger_estop(self):
        self.estop_flag = True

    def trigger_watchdog(self):
        self.watchdog_flag = True

    # ── Ana tick ────────────────────────────────────────────────────────

    def tick(self):
        if self.done:
            return

        # Abort komutu kontrolü
        if self.waypoint_rcvr.pop_abort():
            self._log.warning('Abort alındı → E-stop')
            self._do_terminal(State.E_STOP)
            return

        # E-stop / Watchdog tetikleyicileri
        _terminal = {State.E_STOP, State.WATCHDOG_TRIGGERED}
        if self.estop_flag and self.state not in _terminal:
            self._do_terminal(State.E_STOP)
            return
        if self.watchdog_flag and self.state not in _terminal:
            self._do_terminal(State.WATCHDOG_TRIGGERED)
            return

        # Görev zaman aşımı (ERC §7.3.2.1: 20 dakika)
        if (self._mission_start_ts is not None
                and (time.time() - self._mission_start_ts) > MISSION_TIMEOUT_S
                and self.state not in (_terminal | {State.MISSION_COMPLETE})):
            self._log.warning('Görev süresi doldu (20dk)')
            self._transition(State.MISSION_COMPLETE)
            return

        handler = getattr(self, f'_s_{self.state.name.lower()}', None)
        if handler:
            handler()

    def _transition(self, new_state: State):
        self._log.info('%s → %s', self.state.name, new_state.name)
        self.state = new_state
        try:
            self._step_zmq.send_string(json.dumps({
                'mission': 'autonomous_mission',
                'step':    new_state.name,
                'done':    new_state == State.MISSION_COMPLETE,
            }))
        except Exception:
            pass

    def _do_terminal(self, terminal: State):
        self._transition(terminal)
        execute_estop(self.bus)
        self.done = True

    def _start_mission(self, wpl: WaypointList) -> None:
        """WaypointList'i FSM dahili durumuna kopyalar."""
        self._waypoint_targets    = list(wpl.targets)
        self._approach_dist       = wpl.approach_dist_m
        self._mission_return_home = wpl.return_home
        if self._mission_start_ts is None:
            self._mission_start_ts = time.time()
            self._log.info('Görev başladı — hedefler: %s', wpl.targets)
        self._transition(State.NAVIGATE_INIT)

    # ════════════════════════════════════════════════════════════════════
    #  Engel kaçınma durumları
    # ════════════════════════════════════════════════════════════════════

    def _s_idle(self):
        # Yeni görev var mı?
        mission = self.waypoint_rcvr.pop_mission()
        if mission:
            self._start_mission(mission)
            return
        # YOLO hazır → engel kaçınma moduna geç
        det, _ = self.yolo.get_results()
        if det is not None:
            self._transition(State.SCAN)

    def _s_scan(self):
        det, seg   = self.yolo.get_results()
        depth_data = self.cameras.get_frame('realsense_depth')
        depth      = depth_data[0] if depth_data else None
        action     = self.planner.decide(det, seg, depth)
        if action == Action.DRIVE:
            self._transition(State.DRIVE)
        else:
            self._turn_dir          = action
            self._post_settle_state = State.SCAN
            self._transition(State.TURN_PREP)

    def _s_turn_prep(self):
        # BLDC'yi ÖNCE durdur (donanım kuralı)
        self.bus.send_bldc(0, 0)
        self.bus.send_bldc(1, 0)
        self._t_turn_start = time.time()
        self._transition(State.TURN)

    def _s_turn(self):
        elapsed_ms = (time.time() - self._t_turn_start) * 1000
        if elapsed_ms < TURN_HOLD_MS:
            if self._turn_dir == Action.TURN_LEFT:
                self.bus.send_step_motors(['L', 'L', 'L', 'L'])
            else:
                self.bus.send_step_motors(['R', 'R', 'R', 'R'])
        else:
            self.bus.send_step_motors(['S', 'S', 'S', 'S'])
            self._t_settle = time.time()
            self._transition(State.SETTLE_5MS)

    def _s_settle_5ms(self):
        """Step → BLDC geçiş güvencesi (≥5ms)."""
        if (time.time() - self._t_settle) * 1000 >= SETTLE_MS:
            self._transition(self._post_settle_state)

    def _s_drive(self):
        self.bus.send_bldc(0, SPEED_FWD)
        self.bus.send_bldc(1, SPEED_FWD)
        self._transition(State.CHECK_OBSTACLE)

    def _s_check_obstacle(self):
        # Her tick'te BLDC komutunu yinele
        self.bus.send_bldc(0, SPEED_FWD)
        self.bus.send_bldc(1, SPEED_FWD)
        det, seg   = self.yolo.get_results()
        depth_data = self.cameras.get_frame('realsense_depth')
        depth      = depth_data[0] if depth_data else None
        action     = self.planner.decide(det, seg, depth)
        if action != Action.DRIVE:
            self._turn_dir          = action
            self._post_settle_state = State.SCAN
            self._transition(State.TURN_PREP)

    # ════════════════════════════════════════════════════════════════════
    #  Navigasyon durumları (ERC §7.3.2.1)
    # ════════════════════════════════════════════════════════════════════

    def _s_navigate_init(self):
        if not self._waypoint_targets:
            if self._mission_return_home:
                self._transition(State.RETURN_HOME)
            else:
                self._transition(State.MISSION_COMPLETE)
            return
        self._current_target    = self._waypoint_targets[0]
        self._aruco_lost_ticks  = 0
        self._search_start_ts   = time.time()
        self._last_scan_turn_ts = time.time()
        self._scan_turn_dir     = Action.TURN_RIGHT
        self._pid.reset()
        self._log.info('Hedef: ArUco ID %d (%.1fm yaklaşma)',
                       self._current_target, self._approach_dist)
        self._transition(State.SEARCH_LANDMARK)

    def _s_search_landmark(self):
        dets = self.aruco.get_detections()
        if self._current_target in dets:
            self._log.info('ArUco ID %d bulundu', self._current_target)
            self._pid.reset()
            self._transition(State.ALIGN_HEADING)
            return

        # Arama zaman aşımı → görev iptal
        if time.time() - self._search_start_ts > SEARCH_TIMEOUT_S:
            self._log.warning('ArUco ID %d %ds içinde bulunamadı → MISSION_COMPLETE',
                              self._current_target, int(SEARCH_TIMEOUT_S))
            self._transition(State.MISSION_COMPLETE)
            return

        # Periyodik tarama dönüşü (step motorlar)
        now = time.time()
        if now - self._last_scan_turn_ts > SEARCH_SCAN_STEP_S:
            self._last_scan_turn_ts = now
            # Yön alternatif
            self._scan_turn_dir = (Action.TURN_LEFT
                                   if self._scan_turn_dir == Action.TURN_RIGHT
                                   else Action.TURN_RIGHT)
            self._turn_dir          = self._scan_turn_dir
            self._post_settle_state = State.SEARCH_LANDMARK
            self._transition(State.TURN_PREP)

    def _s_align_heading(self):
        dets = self.aruco.get_detections()
        if self._current_target not in dets:
            self._aruco_lost_ticks += 1
            if self._aruco_lost_ticks > ARUCO_LOST_TICKS_MAX:
                self._log.warning('ArUco ID %d kayboldu → yeniden arama',
                                  self._current_target)
                self._search_start_ts  = time.time()
                self._aruco_lost_ticks = 0
                self.bus.send_bldc(0, 0)
                self.bus.send_bldc(1, 0)
                self._transition(State.SEARCH_LANDMARK)
            return

        self._aruco_lost_ticks = 0
        det = dets[self._current_target]

        if abs(det.heading_err_rad) < ALIGN_THRESHOLD_RAD:
            self._log.info('Başlık hizalandı (err=%.3frad)', det.heading_err_rad)
            self._transition(State.APPROACH_WAYPOINT)
            return

        # PID ile yerinde döndür (diferansiyel BLDC, step motorlar sabit)
        pid_out    = self._pid.update(det.heading_err_rad)
        left_spd   = int(-pid_out * 0.5)
        right_spd  = int( pid_out * 0.5)
        left_spd   = max(-SPEED_TURN, min(SPEED_TURN, left_spd))
        right_spd  = max(-SPEED_TURN, min(SPEED_TURN, right_spd))
        self.bus.send_bldc(0, left_spd)
        self.bus.send_bldc(1, right_spd)

    def _s_approach_waypoint(self):
        # Önce engel kontrolü
        det, seg   = self.yolo.get_results()
        depth_data = self.cameras.get_frame('realsense_depth')
        depth      = depth_data[0] if depth_data else None
        action     = self.planner.decide(det, seg, depth)
        if action != Action.DRIVE:
            self.bus.send_bldc(0, 0)
            self.bus.send_bldc(1, 0)
            self._turn_dir          = action
            self._post_settle_state = State.SEARCH_LANDMARK
            self._transition(State.TURN_PREP)
            return

        # ArUco hedef kontrolü
        aruco_dets = self.aruco.get_detections()
        if self._current_target not in aruco_dets:
            self._aruco_lost_ticks += 1
            if self._aruco_lost_ticks > ARUCO_LOST_TICKS_MAX:
                self._log.warning('Yaklaşırken ArUco ID %d kayboldu',
                                  self._current_target)
                self._search_start_ts  = time.time()
                self._aruco_lost_ticks = 0
                self.bus.send_bldc(0, 0)
                self.bus.send_bldc(1, 0)
                self._transition(State.SEARCH_LANDMARK)
                return
            # Son komutla ilerlemeye devam et
            self.bus.send_bldc(0, SPEED_FWD)
            self.bus.send_bldc(1, SPEED_FWD)
            return

        self._aruco_lost_ticks = 0
        aruco_det = aruco_dets[self._current_target]

        # Waypoint'e ulaşıldı mı?
        if aruco_det.distance_m <= self._approach_dist:
            self._log.info('Waypoint %d ulaşıldı (d=%.2fm)',
                           self._current_target, aruco_det.distance_m)
            self.bus.send_bldc(0, 0)
            self.bus.send_bldc(1, 0)
            self._transition(State.WAYPOINT_REACHED)
            return

        # PID başlık düzeltmesi + ileri hareket
        # heading_err > 0 → işaretçi sağda → sol BLDC hızlandır → sağa dön
        pid_out   = self._pid.update(aruco_det.heading_err_rad)
        left_spd  = max(-1000, min(1000, int(SPEED_FWD + pid_out)))
        right_spd = max(-1000, min(1000, int(SPEED_FWD - pid_out)))
        self.bus.send_bldc(0, left_spd)
        self.bus.send_bldc(1, right_spd)

    def _s_waypoint_reached(self):
        if self._waypoint_targets:
            self._waypoint_targets.pop(0)

        if self._waypoint_targets:
            self._log.info('Kalan waypoint sayısı: %d', len(self._waypoint_targets))
            self._transition(State.NAVIGATE_INIT)
        elif self._mission_return_home:
            self._log.info('Tüm waypointler tamamlandı → eve dönüş')
            self._return_home_start_ts = None
            self._transition(State.RETURN_HOME)
        else:
            self._transition(State.MISSION_COMPLETE)

    def _s_return_home(self):
        # İlk kez girişte zaman damgası
        if self._return_home_start_ts is None:
            self._return_home_start_ts = time.time()
            self._log.info('Eve dönüş başladı (max %.0fs)', RETURN_HOME_TIMEOUT_S)

        # Zaman aşımı → görevi bitir
        if time.time() - self._return_home_start_ts > RETURN_HOME_TIMEOUT_S:
            self._log.info('Eve dönüş zaman aşımı → MISSION_COMPLETE')
            self.bus.send_bldc(0, 0)
            self.bus.send_bldc(1, 0)
            self._transition(State.MISSION_COMPLETE)
            return

        # Engel kontrolüyle ileri git
        det, seg   = self.yolo.get_results()
        depth_data = self.cameras.get_frame('realsense_depth')
        depth      = depth_data[0] if depth_data else None
        action     = self.planner.decide(det, seg, depth)
        if action != Action.DRIVE:
            self._turn_dir          = action
            self._post_settle_state = State.RETURN_HOME
            self._transition(State.TURN_PREP)
        else:
            self.bus.send_bldc(0, SPEED_FWD)
            self.bus.send_bldc(1, SPEED_FWD)

    def _s_mission_complete(self):
        self.bus.send_bldc(0, 0)
        self.bus.send_bldc(1, 0)
        self.bus.send_step_motors(['S', 'S', 'S', 'S'])
        self._log.info('━━━ GÖREV TAMAMLANDI ━━━')
        try:
            self._step_zmq.send_string(json.dumps({
                'mission': 'autonomous_mission',
                'step':    'MISSION_COMPLETE',
                'done':    True,
            }))
        except Exception:
            pass
        self.done = True

    # Terminal durumlar (execute_estop _do_terminal tarafından çağrıldı)
    def _s_e_stop(self):
        pass

    def _s_watchdog_triggered(self):
        pass


# ═══════════════════════════════════════════════════════════════════════════
#  14.B STANDALONE LAUNCH MANAGER (GUI İÇİN)
# ═══════════════════════════════════════════════════════════════════════════

class StandaloneLaunchManager(Thread):
    """
    Eğer --no-launch-manager verilmediyse, arc_electron arayüzünün 
    port 5560 üzerinden bağlanıp durumu görebilmesi için sahte bir LaunchManager gibi davranır.
    """
    _log = logging.getLogger('otonom.launch')

    def __init__(self, fsm: 'AutonomousFSM'):
        super().__init__(daemon=True, name='StandaloneLaunchManager')
        self.fsm = fsm
        self._stop = Event()
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.setsockopt(zmq.RCVTIMEO, 500)
        self.sock.setsockopt(zmq.LINGER, 0)

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            self.sock.bind(f"tcp://*:{ZMQ_LAUNCH_PORT}")
            self._log.info('Standalone ZMQ REP tcp://*:%d (Arayüz bağlantısı için)', ZMQ_LAUNCH_PORT)
        except Exception as exc:
            self._log.error('Port %d bind edilemedi: %s', ZMQ_LAUNCH_PORT, exc)
            return

        while not self._stop.is_set():
            try:
                raw = self.sock.recv_string()
            except zmq.Again:
                continue
            except Exception:
                break

            try:
                cmd = json.loads(raw)
            except Exception:
                self.sock.send_string(json.dumps({'success': False, 'message': 'Geçersiz JSON'}))
                continue

            action = cmd.get('action', '')
            result = {'success': False, 'message': f'Bilinmeyen: {action}'}

            if action == 'ping':
                result = {
                    'success': True, 'message': 'pong',
                    'timestamp': time.time(),
                    'available_launches': ['autonomous_mission'],
                }
            elif action == 'status':
                result = {
                    'success': True,
                    'launches': {
                        'autonomous_mission': {
                            'running': True,
                            'pid': os.getpid(),
                            'description': 'Otonom Navigasyon (Standalone)',
                            'recent_logs': [],
                        }
                    }
                }
            elif action == 'start':
                result = {'success': True, 'message': 'autonomous_mission AKTIF', 'pid': os.getpid()}
            elif action == 'stop' or action == 'stop_all':
                result = {'success': True, 'message': 'durduruluyor...'}
                self.sock.send_string(json.dumps(result))
                self._log.warning("Arayüzden durdurma komutu geldi, çıkılıyor.")
                self.fsm.trigger_estop()
                self.fsm.done = True
                continue
            elif action == 'step_mark':
                result = {'success': True, 'message': 'step_mark_received'}

            try:
                self.sock.send_string(json.dumps(result))
            except Exception:
                pass

        try:
            self.sock.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
#  15. MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Marmara Rover – Otonom Sürüş v2  (ERC 2026 Uyumlu)')

    # YOLO
    parser.add_argument('--detect-pt',     default=None,
                        help='Detection YOLO .pt dosya yolu')
    parser.add_argument('--seg-pt',        default=None,
                        help='Segmentation YOLO .pt dosya yolu')
    parser.add_argument('--no-yolo',       action='store_true',
                        help='YOLO çalıştırma (geliştirme modu)')
    parser.add_argument('--yolo-conf',     type=float, default=YOLO_CONF_THRESH)
    parser.add_argument('--fake-obstacle', choices=['none', 'left', 'right'],
                        default='none',
                        help='Sahte engel (geliştirme)')

    # Motor portları
    parser.add_argument('--bldc1-port',    default='/dev/ttyUSB0',
                        help='BLDC anakart 1 UART portu')
    parser.add_argument('--bldc2-port',    default='/dev/ttyUSB1',
                        help='BLDC anakart 2 UART portu')
    parser.add_argument('--step-port',     default='/dev/ttyACM0',
                        help='Step motor Arduino UART portu')
    parser.add_argument('--bus',           choices=['uart', 'can'], default='uart',
                        help='Motor haberleşme: uart (varsayılan) veya can (TODO)')

    # Ağ / Diğer
    parser.add_argument('--jpeg-quality',   type=int, default=80)
    parser.add_argument('--telemetry-host', default='127.0.0.1',
                        help='Sensors.py ZMQ host')
    parser.add_argument('--waypoint-host',  default='127.0.0.1',
                        help='WaypointReceiver ZMQ host')

    # rover_launcher.py tarafından başlatıldığında otomatik eklenir
    # (port 5560 çakışmasını önler — bu script kullanmaz, yok sayılır)
    parser.add_argument('--no-launch-manager', action='store_true',
                        help='rover_launcher.py subprocess modu (port 5560 kullanılmaz)')

    args = parser.parse_args()

    log.info('=' * 70)
    log.info('  MARMARA ROVER – OTONOM SÜRÜŞ  v2  (ERC 2026)')
    log.info('=' * 70)
    log.info('  Bus      : %s', args.bus)
    log.info('  BLDC1    : %s', args.bldc1_port)
    log.info('  BLDC2    : %s', args.bldc2_port)
    log.info('  Step     : %s', args.step_port)
    log.info('  YOLO det : %s', args.detect_pt or '—')
    log.info('  YOLO seg : %s', args.seg_pt    or '—')
    log.info('  No-YOLO  : %s', args.no_yolo)
    log.info('  Fake obs : %s', args.fake_obstacle)
    log.info('  ArUco    : DICT_5X5_100  ID 51-64  150mm')
    log.info('-' * 70)

    # ── Motor Bus ────────────────────────────────────────────────────────
    # CANMotorBus.__init__ raises SystemExit if selected — handled cleanly
    if args.bus == 'can':
        bus: MotorBus = CANMotorBus()
    else:
        bus = UARTMotorBus(args.bldc1_port, args.bldc2_port, args.step_port)

    bus.capture_home(timeout=3.0)

    # ── Telemetri ────────────────────────────────────────────────────────
    telemetry = TelemetrySubscriber(host=args.telemetry_host)
    telemetry.start()

    # ── ROS2 + CameraHub ─────────────────────────────────────────────────
    rclpy.init()
    cameras  = CameraHub()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(cameras)
    Thread(target=executor.spin, daemon=True, name='ROS2Spin').start()

    # ── ZMQ Step Publisher (5561) ─────────────────────────────────────────
    zmq_ctx  = zmq.Context.instance()
    step_pub = zmq_ctx.socket(zmq.PUB)
    step_pub.setsockopt(zmq.SNDHWM, 4)
    try:
        step_pub.bind(f'tcp://*:{ZMQ_STEP_PORT}')
        log.info('ZMQ step PUB :%d', ZMQ_STEP_PORT)
    except zmq.ZMQError as exc:
        log.warning('ZMQ step bind hatası: %s', exc)

    # ── YOLO Worker ───────────────────────────────────────────────────────
    yolo = YoloWorker(cameras,
                      detect_pt     = args.detect_pt,
                      seg_pt        = args.seg_pt,
                      no_yolo       = args.no_yolo,
                      fake_obstacle = args.fake_obstacle,
                      conf          = args.yolo_conf)
    yolo.start()

    # ── ArUco Worker ──────────────────────────────────────────────────────
    aruco = ArucoWorker(cameras)
    aruco.start()

    # ── Planner & Waypoint Receiver ───────────────────────────────────────
    planner      = AvoidancePlanner()
    waypoint_rcvr = WaypointReceiver(host=args.waypoint_host)
    waypoint_rcvr.start()

    # ── FSM ───────────────────────────────────────────────────────────────
    fsm = AutonomousFSM(bus, cameras, yolo, aruco, planner, waypoint_rcvr, step_pub)

    # ── Overlay Publisher ─────────────────────────────────────────────────
    overlay = OverlayPublisher(cameras, yolo, aruco, telemetry, args.jpeg_quality)
    overlay.set_step_pub(step_pub)
    overlay.set_fsm(fsm)
    overlay.start()

    # ── Watchdog ──────────────────────────────────────────────────────────
    watchdog = Watchdog(fsm, telemetry, cameras)
    watchdog.start()

    # ── Standalone Launch Manager ─────────────────────────────────────────
    launch_mgr = None
    if not args.no_launch_manager:
        launch_mgr = StandaloneLaunchManager(fsm)
        launch_mgr.start()

    # ── Sinyal İşleyicisi ────────────────────────────────────────────────
    def _on_signal(sig, _frame):
        log.warning('Sinyal %d alındı → E-stop tetikleniyor', sig)
        fsm.trigger_estop()

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    log.info('[HAZIR] FSM başlıyor @%dHz  |  Ctrl+C = E-stop', FSM_TICK_HZ)
    log.info('Waypoint göndermek için: ZMQ PUB → tcp://localhost:%d', ZMQ_WAYPOINT_PORT)

    # ── Ana FSM Döngüsü ───────────────────────────────────────────────────
    tick_interval = 1.0 / FSM_TICK_HZ
    while not fsm.done:
        t0 = time.monotonic()
        fsm.tick()
        elapsed = time.monotonic() - t0
        rem = tick_interval - elapsed
        if rem > 0:
            time.sleep(rem)

    log.info('FSM tamamlandı — kapatılıyor...')

    # ── Temiz Kapatma ────────────────────────────────────────────────────
    watchdog.stop()
    aruco.stop()
    yolo.stop()
    overlay.stop()
    telemetry.stop()
    waypoint_rcvr.stop()
    if launch_mgr:
        launch_mgr.stop()

    executor.shutdown(wait=False)
    cameras.destroy_node()
    rclpy.shutdown()

    bus.close()
    step_pub.close()
    log.info('Kapatıldı.')


if __name__ == '__main__':
    main()
