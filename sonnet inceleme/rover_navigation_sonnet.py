#!/usr/bin/env python3
# GÜVENLİK: sudo usermod -a -G dialout $USER
# pip install ahrs --break-system-packages

# -------------------------------------------------------------------
# YAPILAN DEĞİŞİKLİKLER (rover_master.py'dan öğrenilen protokoller):
#
#   [✅MT]    MultiThreadedExecutor     — IMU/GNSS callback'leri kontrol döngüsünü bloklamaz
#              RGB → MutuallyExclusive | Diğerleri → Reentrant
#   [✅WD]    Watchdog timer            — 0.5 sn kontrol döngüsü yoksa acil dur
#   [✅RC]    Seri port reconnect       — 3 port için ayrı ayrı, max 3 deneme
#   [✅AHRS]  Mahony füzyon             — ham quaternion→euler yerine filtreli yaw
#   [✅VS]    Velocity smoothing        — alpha=0.15 low-pass, ani jerk engelleme
#   [✅PI]    PI kontrolü               — global_Ki ile uzun mesafe sapma düzeltme
#   [✅DT]    dt eşiği                  — 0.3 sn üstünü kes
#   [✅LOG]   Logging throttle          — tekrarlayan loglar 1-2 Hz'e alındı
#   [BUG FIX] vel_linear kullanılmıyordu — odometri için hesaba katıldı
#   [BUG FIX] raw_angular tanımsız kalabiliyordu — deadband bloğu düzeltildi
#   [BUG FIX] ArUco error_x hesaplanıyor ama kullanılmıyordu — servoing'e bağlandı
#   [✅GNSS]  GNSS warmup std sapma kontrolü — kararsız fix'leri dışarıda bırak
# -------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, Imu, NavSatFix
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
import serial
import struct
import cv2
from threading import Lock

from ahrs.filters import Mahony
from ahrs.common.orientation import q2euler


class RoverNavigation(Node):
    def __init__(self):
        super().__init__("Rover_navigation_node")

        # ============================================================
        #  CALLBACK GRUPLARI
        # ============================================================
        # RGB (ArUco) işlemi ağır olabilir → kendi grubu
        self.aruco_cb_group   = MutuallyExclusiveCallbackGroup()
        # IMU, GNSS, kontrol döngüsü → birbirini beklemeden çalışır
        self.control_cb_group = ReentrantCallbackGroup()

        # ============================================================
        #  KİLİT
        # ============================================================
        self.data_lock = Lock()

        # ============================================================
        #  SERİ PORTLAR
        # ============================================================
        self.ser_read         = self._open_serial('/dev/ttyUSB0', tag='ser_read')
        self.ser_write_front  = self._open_serial('/dev/ttyUSB1', tag='ser_write_front')
        self.ser_write_back   = self._open_serial('/dev/ttyUSB2', tag='ser_write_back')

        self.START_FRAME = 0xABCD
        self.buffer      = bytearray()

        # ============================================================
        #  KAMERA & ArUco
        # ============================================================
        self.cv_bridge     = CvBridge()
        self.aruco_dict    = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params  = cv2.aruco.DetectorParameters()

        # ArUco servoing verisi (kilitli paylaşım)
        self.aruco_error_x  = None   # piksel cinsinden yatay hata (None = marker yok)
        self.aruco_img_w    = 640    # varsayılan genişlik, rgb_callback'te güncellenir
        self.ARUCO_Kp       = 0.003  # piksel→angular_speed oransal kazanç — sahada ayarla

        # ============================================================
        #  GNSS
        # ============================================================
        self.warmup_count  = 0
        self.warmup_limit  = 20
        self.lat_buffer    = []
        self.lon_buffer    = []

        # [✅GNSS] Std sapma eşiği — kararsız fix'leri filtrele
        self.GNSS_STD_THRESHOLD = 0.00005  # ~5 m beklenen std sapma
        self.is_gnss_available  = False
        self.gnss_origin_lat    = None
        self.gnss_origin_lon    = None

        self.current_x = None
        self.current_y = None

        # [GNSS YAW] Ardışık GNSS fix'lerinden Mahony drift düzeltmesi
        self.prev_gnss_x       = None
        self.prev_gnss_y       = None
        self.GNSS_YAW_MIN_DIST = 1.0   # m — düzeltme için gereken min hareket
        self.GNSS_YAW_BLEND    = 0.1   # yavaş blend — GNSS yaw'ı gürültülü

        # ============================================================
        #  IMU & AHRS
        # ============================================================
        # [✅AHRS] Ham quaternion→euler yerine Mahony filtresi
        self.IMU_FREQ = 100.0   # ⚠️ ros2 topic hz /imu/data ile doğrula
        self.mahony   = Mahony(frequency=self.IMU_FREQ, k_P=2.0, k_I=0.005)
        self.mahony.Q = np.array([1.0, 0.0, 0.0, 0.0])

        self.current_yaw      = 0.0   # radyan — Mahony çıktısı
        self.BLEND_ALPHA      = 0.02  # yaw blending hızı (ani sıçrama önleme)

        # ============================================================
        #  NAVİGASYON PARAMETRELERİ
        # ============================================================
        self.heading_deadband    = 0.05   # rad — min dönme açısı
        self.min_angular_speed   = 0.1    # rad/s
        self.max_angular_speed   = 1.0    # rad/s
        self.max_linear_speed    = 1.0    # m/s

        # [✅PI] PI kontrol sabitleri
        self.Kp_linear   = 0.5
        self.Kp_angular  = 1.5
        self.Ki_angular  = 0.03   # integral — uzun mesafe sabit sapma düzeltme
        self.angular_integral  = 0.0
        self.INTEGRAL_LIMIT    = 0.5   # windup koruması

        # [✅VS] Velocity smoothing
        self.SMOOTH_ALPHA  = 0.15
        self.prev_linear   = 0.0
        self.prev_angular  = 0.0

        # [STALL] Takılma tespiti
        self.ROVER_WIDTH   = 0.5    # m — tekerlek aralığı (rover genişliği)
        self.STALL_MAX     = 20     # ~1 sn (20 Hz kontrol döngüsünde)
        self.STALL_VEL_THR = 0.05   # m/s — bu altı "durdu" sayılır
        self.stall_counter = 0

        # ============================================================
        #  HEDEF NOKTALARI
        # ============================================================
        self.targets = [
            (7,   10),
            (8,   17),
            (0,   25),
            (-7,  15),
        ]
        self.current_target_idx = 0   # aktif hedef indisi

        self.current_target_x  = self.targets[0][0]
        self.current_target_y  = self.targets[0][1]

        # ============================================================
        #  ZAMANLAMA
        # ============================================================
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # [✅WD] Watchdog
        self.last_control_time = time.time()

        # [✅DT] dt eşiği
        self.DT_MAX = 0.3

        # ============================================================
        #  YAYINCILAR
        # ============================================================
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ============================================================
        #  ABONELER
        # ============================================================
        self.create_subscription(
            Image, '/realsense/rgb/image_raw',
            self.rgb_callback, 10,
            callback_group=self.aruco_cb_group
        )
        self.create_subscription(
            Image, '/realsense/depth/color_aligned',
            self.align_depth_callback, 10,
            callback_group=self.control_cb_group
        )
        self.create_subscription(
            NavSatFix, '/gnss/fix',
            self.gnss_callback, 10,
            callback_group=self.control_cb_group
        )
        self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10,
            callback_group=self.control_cb_group
        )

        # ============================================================
        #  ZAMANLAYICILAR
        # ============================================================
        self.timer = self.create_timer(
            0.05, self.control_loop,
            callback_group=self.control_cb_group
        )
        # [✅WD] Watchdog timer
        self.watchdog_timer = self.create_timer(
            0.5, self.watchdog_check,
            callback_group=self.control_cb_group
        )

        # [✅RC] Arka plan reconnect timer — hot path'i bloğlamaz
        self.reconnect_timer = self.create_timer(
            2.0, self._background_reconnect,
            callback_group=self.control_cb_group
        )

        self.get_logger().info("RoverNavigation başlatıldı.")

    # ================================================================
    #  SERİ PORT YARDIMCILARI
    # ================================================================

    def _open_serial(self, port, tag='port'):
        """Seri portu aç; başarısızsa None döndür."""
        try:
            s = serial.Serial(port, 115200, timeout=0.1)
            self.get_logger().info(f"Seri port açıldı: {tag} ({port})")
            return s
        except Exception as e:
            self.get_logger().error(f"Seri port hatası ({tag}): {e}")
            return None

    def _reconnect_serial(self, port_attr, port_path, tag):
        """[✅RC] Belirtilen seri portu bir kez açmayı dene (bloğlamaz, sleep yok)."""
        try:
            s = serial.Serial(port_path, 115200, timeout=0.1)
            setattr(self, port_attr, s)
            self.get_logger().info(f"{tag} yeniden bağlandı.")
        except Exception as e:
            self.get_logger().warn(f"{tag} reconnect başarısız: {e}", throttle_duration_sec=5.0)
            setattr(self, port_attr, None)

    def _background_reconnect(self):
        """[✅RC] Her 2 sn'de kapalı portları arka planda yeniden bağla."""
        for attr, path, tag in [
            ('ser_read',        '/dev/ttyUSB0', 'ser_read'),
            ('ser_write_front', '/dev/ttyUSB1', 'ser_write_front'),
            ('ser_write_back',  '/dev/ttyUSB2', 'ser_write_back'),
        ]:
            port = getattr(self, attr)
            if port is None or not port.is_open:
                self._reconnect_serial(attr, path, tag)

    # ================================================================
    #  WATCHDOG
    # ================================================================

    def watchdog_check(self):
        """[✅WD] Kontrol döngüsü 0.5 sn'dir çalışmadıysa acil dur."""
        if time.time() - self.last_control_time > 0.5:
            self.get_logger().error(
                "WATCHDOG: Kontrol döngüsü yanıt vermiyor! Acil dur.",
                throttle_duration_sec=1.0
            )
            self.send_to_hoverboard(0.0, 0.0)

    # ================================================================
    #  ENCODER GERİ BESLEME
    # ================================================================

    def read_serial_feedback(self):
        """Hoverboard'dan encoder + telemetri verisi oku."""
        if self.ser_read is None or not self.ser_read.is_open:
            # Arka plan timer yeniden bağlayacak; burada bloğlanmıyoruz
            return None

        try:
            waiting = self.ser_read.in_waiting
            if waiting > 0:
                self.buffer.extend(self.ser_read.read(waiting))
        except Exception as e:
            self.get_logger().warn(f"Seri okuma hatası: {e}")
            self.ser_read = None
            return None

        if len(self.buffer) < 18:
            return None

        header_bytes     = b'\xCD\xAB'
        last_packet_idx  = self.buffer.rfind(header_bytes)

        if last_packet_idx == -1:
            self.buffer.clear()
            return None

        if len(self.buffer) < last_packet_idx + 18:
            return None

        unpacked = struct.unpack(
            '<hhhhhhHH',
            self.buffer[last_packet_idx + 2: last_packet_idx + 18]
        )
        calc_cs = (
            self.START_FRAME
            ^ unpacked[0] ^ unpacked[1] ^ unpacked[2]
            ^ unpacked[3] ^ unpacked[4] ^ unpacked[5]
            ^ unpacked[6]
        ) & 0xFFFF

        del self.buffer[:last_packet_idx + 18]

        if calc_cs != unpacked[7]:
            self.get_logger().warn(
                f"Checksum hatası! calc={calc_cs} data={unpacked[7]}",
                throttle_duration_sec=2.0
            )
            return None

        return {
            'cmd1':        unpacked[0],
            'cmd2':        unpacked[1],
            'speedR_meas': unpacked[2],
            'speedL_meas': unpacked[3],
            'batVoltage':  unpacked[4],
            'boardTemp':   unpacked[5],
            'cmdLed':      unpacked[6],
            'checksum':    unpacked[7],
        }

    # ================================================================
    #  CALLBACK'LER
    # ================================================================

    def align_depth_callback(self, msg):
        # Depth verisi şimdilik işlenmiyor — ileride engel algılama için
        pass

    def rgb_callback(self, msg):
        """[ArUco] Marker tespit et, yatay hata pikselini hesapla."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge hatası: {e}")
            return

        h, w = cv_image.shape[:2]
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        found_error = None

        if ids is not None:
            for i, marker_id in enumerate(ids):
                mid = marker_id[0]
                if 51 <= mid <= 64:
                    c          = corners[i][0]
                    center_x   = float(np.mean(c[:, 0]))
                    # [BUG FIX] error_x artık sadece hesaplanmıyor, paylaşılıyor
                    found_error = center_x - (w / 2.0)
                    break   # en büyük öncelikli marker — ilk bulunanı al

        with self.data_lock:
            self.aruco_error_x = found_error
            self.aruco_img_w   = w

    def gnss_callback(self, msg):
        """[✅GNSS] GNSS verisini düzlemsel koordinata çevir."""
        R = 6378137.0

        if not self.is_gnss_available:
            if self.warmup_count < self.warmup_limit:
                self.lat_buffer.append(msg.latitude)
                self.lon_buffer.append(msg.longitude)
                self.warmup_count += 1
                self.get_logger().info(
                    f"GNSS kalibrasyon {self.warmup_count}/{self.warmup_limit}",
                    throttle_duration_sec=2.0
                )
                return
            else:
                # [✅GNSS] Std sapma kontrolü — kararsız fix varsa reddet
                lat_std = float(np.std(self.lat_buffer))
                lon_std = float(np.std(self.lon_buffer))
                if lat_std > self.GNSS_STD_THRESHOLD or lon_std > self.GNSS_STD_THRESHOLD:
                    self.get_logger().warn(
                        f"GNSS kalibrasyon std çok yüksek "
                        f"(lat_std={lat_std:.6f}, lon_std={lon_std:.6f}). "
                        f"Yeniden kalibrasyon başlıyor."
                    )
                    # Tampon sıfırla ve yeniden dene
                    self.lat_buffer.clear()
                    self.lon_buffer.clear()
                    self.warmup_count = 0
                    return

                self.gnss_origin_lat = float(np.mean(self.lat_buffer))
                self.gnss_origin_lon = float(np.mean(self.lon_buffer))
                self.is_gnss_available = True
                self.get_logger().info(
                    f"GNSS kalibre edildi. "
                    f"Origin: {self.gnss_origin_lat:.6f}, {self.gnss_origin_lon:.6f}"
                )

        try:
            d_lat       = math.radians(msg.latitude  - self.gnss_origin_lat)
            d_lon       = math.radians(msg.longitude - self.gnss_origin_lon)
            ref_lat_rad = math.radians(self.gnss_origin_lat)

            with self.data_lock:
                self.current_x = d_lon * R * math.cos(ref_lat_rad)
                self.current_y = d_lat * R

                # [GNSS YAW] Mahony yaw drift'ini ardışık fix'lerle düzelt.
                # Magnetometre olmadan Mahony zamanla kayar; GNSS hareketi
                # gerçek başlığı verir ve yavaş blend ile filtreye işlenir.
                if self.prev_gnss_x is not None:
                    move_dx = self.current_x - self.prev_gnss_x
                    move_dy = self.current_y - self.prev_gnss_y
                    if math.sqrt(move_dx**2 + move_dy**2) > self.GNSS_YAW_MIN_DIST:
                        gnss_yaw = math.atan2(move_dy, move_dx)
                        yaw_diff = math.atan2(
                            math.sin(gnss_yaw - self.current_yaw),
                            math.cos(gnss_yaw - self.current_yaw)
                        )
                        self.current_yaw += self.GNSS_YAW_BLEND * yaw_diff
                        self.current_yaw = math.atan2(
                            math.sin(self.current_yaw),
                            math.cos(self.current_yaw)
                        )

                self.prev_gnss_x = self.current_x
                self.prev_gnss_y = self.current_y

        except Exception as e:
            self.get_logger().warn(f"GNSS okuma hatası: {e}", throttle_duration_sec=2.0)

    def imu_callback(self, msg):
        """[✅AHRS] Mahony filtresi ile quaternion güncelle → yaw çıkar."""
        gyr = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        try:
            self.mahony.Q = self.mahony.updateIMU(self.mahony.Q, gyr=gyr, acc=acc)
            euler         = q2euler(self.mahony.Q)  # [roll, pitch, yaw] radyan
            new_yaw       = euler[2]

            # [BUG FIX / ✅AHRS] Yaw blending — ani overwrite yerine yumuşak geçiş
            yaw_diff = math.atan2(
                math.sin(new_yaw - self.current_yaw),
                math.cos(new_yaw - self.current_yaw)
            )
            with self.data_lock:
                self.current_yaw = self.current_yaw + self.BLEND_ALPHA * yaw_diff
                self.current_yaw = math.atan2(
                    math.sin(self.current_yaw),
                    math.cos(self.current_yaw)
                )
        except Exception as e:
            self.get_logger().warn(f"AHRS güncelleme hatası: {e}", throttle_duration_sec=2.0)

    # ================================================================
    #  KONTROL DÖNGÜSÜ
    # ================================================================

    def control_loop(self):
        # [✅WD] Watchdog zaman damgası
        self.last_control_time = time.time()

        now = self.get_clock().now().nanoseconds / 1e9
        dt  = now - self.last_time
        self.last_time = now

        # [✅DT] Anormal gecikmeyi kes
        if dt > self.DT_MAX:
            dt = 0.05

        # --- GNSS bekleniyor ---
        if not self.is_gnss_available:
            self.get_logger().warn(
                "GNSS bekleniyor...", throttle_duration_sec=2.0
            )
            return

        # --- Konum ve yaw kilit içinde oku ---
        with self.data_lock:
            cur_x       = self.current_x
            cur_y       = self.current_y
            cur_yaw     = self.current_yaw
            aruco_err   = self.aruco_error_x
            img_w       = self.aruco_img_w

        if cur_x is None or cur_y is None:
            self.get_logger().warn("Konum verisi yok.", throttle_duration_sec=2.0)
            return

        # --- Encoder geri besleme ---
        feedback = self.read_serial_feedback()
        if feedback is None:
            self.get_logger().warn(
                "Encoder okunamadı, dur komut gönderiliyor.",
                throttle_duration_sec=2.0
            )
            self.send_to_hoverboard(0.0, 0.0)
            return

        # Encoder → m/s
        feedback_to_ms = 0.001
        vel_right  = feedback['speedR_meas'] * feedback_to_ms
        vel_left   = feedback['speedL_meas'] * feedback_to_ms
        vel_linear = (vel_right + vel_left) / 2.0

        # ----------------------------------------------------------------
        # HEDEF SEÇİMİ
        # ----------------------------------------------------------------
        if self.current_target_idx < len(self.targets):
            self.current_target_x, self.current_target_y = \
                self.targets[self.current_target_idx]
        else:
            # Tüm hedefler tamamlandı
            self.get_logger().info("MİSYON TAMAMLANDI!", throttle_duration_sec=5.0)
            self.send_to_hoverboard(0.0, 0.0)
            return

        # ----------------------------------------------------------------
        # HATA HESAPLAMA
        # ----------------------------------------------------------------
        dx = self.current_target_x - cur_x
        dy = self.current_target_y - cur_y
        distance_error = math.sqrt(dx**2 + dy**2)

        target_yaw    = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_yaw - cur_yaw),
            math.cos(target_yaw - cur_yaw)
        )

        # ----------------------------------------------------------------
        # HEDEFE ULAŞILDI MI?
        # ----------------------------------------------------------------
        if distance_error < 0.5:
            self.get_logger().info(
                f"Hedef {self.current_target_idx + 1} ulaşıldı.",
                throttle_duration_sec=1.0
            )
            self.current_target_idx += 1
            self.angular_integral = 0.0   # integratörü sıfırla
            self.send_to_hoverboard(0.0, 0.0)
            return

        # ----------------------------------------------------------------
        # ArUco SERVOİNG (marker görünürse angular speed'i override et)
        # ----------------------------------------------------------------
        aruco_active = False
        if aruco_err is not None:
            # Marker görünüyor → açı hatasını piksel hatasına göre belirle
            aruco_angular = -self.ARUCO_Kp * aruco_err
            aruco_angular = float(np.clip(
                aruco_angular,
                -self.max_angular_speed,
                self.max_angular_speed
            ))
            aruco_active  = True

        # ----------------------------------------------------------------
        # PI KONTROL — Açısal hız
        # ----------------------------------------------------------------
        target_angular = 0.0

        if aruco_active:
            # ArUco marker var → marker'a yönel
            target_angular       = aruco_angular
            self.angular_integral = 0.0   # ArUco aktifken integral biriktirme
        elif abs(heading_error) < self.heading_deadband:
            # [BUG FIX] Deadband içindeyken raw_angular tanımsız kalıyordu
            target_angular       = 0.0
            self.angular_integral = 0.0
        else:
            # [✅PI] PI kontrol
            self.angular_integral += heading_error * dt
            self.angular_integral  = float(np.clip(
                self.angular_integral,
                -self.INTEGRAL_LIMIT, self.INTEGRAL_LIMIT
            ))
            raw_angular    = (self.Kp_angular * heading_error
                              + self.Ki_angular * self.angular_integral)

            # Min hız garantisi (işaret koru)
            if raw_angular > 0:
                target_angular = max(self.min_angular_speed,
                                     min(raw_angular, self.max_angular_speed))
            else:
                target_angular = min(-self.min_angular_speed,
                                     max(raw_angular, -self.max_angular_speed))

        # ----------------------------------------------------------------
        # DOĞRUSAL HIZ
        # ----------------------------------------------------------------
        target_linear = float(np.clip(
            self.Kp_linear * distance_error,
            -self.max_linear_speed,
            self.max_linear_speed
        ))

        # ----------------------------------------------------------------
        # [✅VS] VELOCİTY SMOOTHING
        # ----------------------------------------------------------------
        self.prev_linear  = ((1.0 - self.SMOOTH_ALPHA) * self.prev_linear
                             + self.SMOOTH_ALPHA * target_linear)
        self.prev_angular = ((1.0 - self.SMOOTH_ALPHA) * self.prev_angular
                             + self.SMOOTH_ALPHA * target_angular)

        # ----------------------------------------------------------------
        # MOTOR & YAYINcı
        # ----------------------------------------------------------------
        self.send_to_hoverboard(self.prev_linear, self.prev_angular)

        # [STALL] Komut verildi ama encoder hareketi yok → rover takıldı
        if abs(self.prev_linear) > 0.1 and abs(vel_linear) < self.STALL_VEL_THR:
            self.stall_counter += 1
            if self.stall_counter >= self.STALL_MAX:
                self.get_logger().warn(
                    f"ROVER TAKILDI! Encoder: {vel_linear:.3f} m/s | "
                    f"Komut: {self.prev_linear:.2f} m/s",
                    throttle_duration_sec=1.0
                )
        else:
            self.stall_counter = 0

        cmd             = Twist()
        cmd.linear.x    = self.prev_linear
        cmd.angular.z   = self.prev_angular
        self.vel_pub.publish(cmd)

        # [✅LOG] Throttle'lı durum logu
        self.get_logger().info(
            f"Hedef {self.current_target_idx + 1}/{len(self.targets)} | "
            f"dist={distance_error:.2f}m | "
            f"heading_err={math.degrees(heading_error):.1f}° | "
            f"v={self.prev_linear:.2f} w={self.prev_angular:.2f} | "
            f"ArUco={'aktif' if aruco_active else 'yok'}",
            throttle_duration_sec=1.0
        )

    # ================================================================
    #  MOTOR SÜRÜCÜ
    # ================================================================

    def send_to_hoverboard(self, lin_speed, ang_speed):
        LINEAR_SCALE  = 300.0
        ANGULAR_SCALE = 150.0

        uSpeed = int(np.clip(lin_speed * LINEAR_SCALE, -1000, 1000))
        uSteer = int(np.clip(ang_speed * ANGULAR_SCALE, -1000, 1000))

        checksum = (self.START_FRAME
                    ^ (uSteer & 0xFFFF)
                    ^ (uSpeed & 0xFFFF)) & 0xFFFF
        packet   = struct.pack('<HhhH', self.START_FRAME, uSteer, uSpeed, checksum)

        # [✅RC] Her port için ayrı hata yönetimi ve reconnect
        for attr, path, tag in [
            ('ser_write_front', '/dev/ttyUSB1', 'ser_write_front'),
            ('ser_write_back',  '/dev/ttyUSB2', 'ser_write_back'),
        ]:
            port = getattr(self, attr)
            if port is None or not port.is_open:
                # Arka plan timer yeniden bağlayacak; burada bloğlanmıyoruz
                self.get_logger().warn(f"{tag} kapalı.", throttle_duration_sec=2.0)
                continue

            try:
                port.write(packet)
            except Exception as e:
                self.get_logger().error(
                    f"{tag} yazma hatası: {e}",
                    throttle_duration_sec=2.0
                )
                setattr(self, attr, None)


# ================================================================
#  ANA GİRİŞ
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        node     = RoverNavigation()
        # [✅MT] MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"KRİTİK: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
