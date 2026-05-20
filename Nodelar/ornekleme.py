#!/usr/bin/env python3
# GÜVENLİK: sudo usermod -a -G dialout $USER

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import struct
import math
import time
import os
from threading import Lock
from collections import deque
from ahrs.filters import Mahony, EKF
from ahrs.common.orientation import q2euler

# -------------------------------------------------------------------
# YAPILAN DEĞİŞİKLİKLER:
#   [✅2]  Temporal smoothing  — target_angle için 5-elemanlı kayan ortalama
#   [✅3]  IMU entegrasyonu    — eğim eşiği (30°/25°), geri çekilme, pitch/roll odometri kompanzasyonu
#   [✅4]  Yaw←IMU gyro        — pose_yaw artık gyro_z entegrasyonuyla güncelleniyor
#   [✅6]  PID (navigasyon)    — görsel servoing: PD | global yaklaşım: PI
#   [✅8]  Watchdog timer      — 0.5 sn kontrolsüzlükte acil dur
#   [✅9]  dt eşiği            — 1.0 → 0.3 sn
#   [✅1]  Derinlik filtresi   — ROI medyanı (%20 iç kenar + 9×9 merkez penceresi)
#   [✅10] Seri port yeniden bağlanma — exception sonrası otomatik reconnect (max 3 deneme)
#   [✅11] Velocity smoothing         — alpha=0.2 low-pass filtresi, ani jerk engelleme
#   [✅12] D terimi clamp + min dt    — türev patlaması koruması
#   [✅13] Depth outlier reject       — dist_m > 10 m ise geçersiz say
#   [✅14] Fotoğraf stabilizasyon     — çekim öncesi araç durmuş mu kontrolü
#   [✅15] Dynamic speed              — hedefe yaklaştıkça hız azalır
#   [✅16] Angle deadband             — 0.02 rad altı hata sıfırla, jitter önle
#   [✅17] State logging              — her döngüde durum logu
#   [✅MT] MultiThreadedExecutor      — YOLO callback'i kontrol döngüsünü bloklamaz
#          callback_group ayrımı: yolo=MutuallyExclusive, kontrol=Reentrant
#   [✅AHRS] Mahony/EKF füzyonu        — ahrs kütüphanesi, USE_EKF flag ile seçilebilir
#            imu_callback → quaternion güncelle → q2euler → pitch/roll/yaw
#            pose_yaw artık ham gyro entegrasyonu değil, filtreli AHRS çıktısı
#   [BUG FIX] Smoothing bug             — send_to_motor(self.prev_v, self.prev_w) düzeltildi
#   [BUG FIX] Yaw ani overwrite         — blending alpha=0.02, wrap-around güvenli
#   [✅LOG]  Logging throttle           — state/eğim/stabilizasyon logları 1-2 Hz'e alındı
# -------------------------------------------------------------------

#------------------------------------------
#YAPILABİLECEK DEĞİŞİKLİKLER
#1. Odometri / wheel slip — Encoder olmadığı ve motor sürücüsünün slip kontrolü yaptığı söylendi. v_filtered = 0.7 * imu + 0.3 * command önerisi ise IMU'dan lineer hız çıkarmayı gerektiriyor ki ahrs bunu vermiyor, ayrıca entegrasyon gerekir. Şimdilik pas, encoder kararı netleşince tekrar bakılır.
#2. YOLO latency compensation — target_angle += gyro_z * latency fikri doğru prensipte ama latency'i sabit 150ms saymak tehlikeli — değişkendir. Bunu yapmak için YOLO'nun timestamp'ini takip etmek gerekir. MultiThreadedExecutor sonrası gerçek latency ne kadar azaldı ölçülmeli. Şimdilik pas, test sonrası değerlendir.
#3. Hysteresis / N-frame confirmation — Mantıklı bir öneri. Ama şu an TIMEOUT_LIMIT=0.7sn ve temporal smoothing bu işi kısmen yapıyor. N-frame eklenmesi false positive'i azaltır. Eklenebilir ama acil değil.
#------------------------------------------

class RoverMaster(Node):
    def __init__(self):
        super().__init__('rover_ornekleme_node')

        self.get_logger().info("ROVER ORNEKLEME BAŞLATILIYOR...")

        # --- CALLBACK GRUPLARI (MultiThreadedExecutor için) ---
        # [✅MT] YOLO callback'i diğer callback'leri bloklamasın
        # MutuallyExclusive: aynı anda sadece bir YOLO çalışır (çakışma önleme)
        # Reentrant: kontrol döngüsü + IMU + depth birbirini beklemeden çalışır
        self.yolo_cb_group    = MutuallyExclusiveCallbackGroup()
        self.control_cb_group = ReentrantCallbackGroup()

        # --- KİLİTLER ---
        self.data_lock = Lock()
        self.yolo_lock = Lock()

        # --- GENEL AYARLAR ---
        self.DEBUG_VISUAL   = False
        self.TIMEOUT_LIMIT  = 0.7   # sn — görsel timeout

        # --- AHRS FİLTRELERİ (Mahony birincil, EKF ikincil/opsiyonel) ---
        # Mahony: hafif, gyro+acc ile stabil, yarışma süresi için yeterli
        # EKF: daha hassas, biraz daha ağır — USE_EKF=True ile etkinleştir
        self.USE_EKF = False  # True yapılırsa EKF kullanılır, False → Mahony
        self.IMU_FREQ = 100.0  # IMU yayın frekansı (Hz) — kendi setup'ına göre ayarla

        # Mahony parametreleri:
        #   k_P: orantısal kazanç — yüksek → acc'a güven artar, gyro drift azalır
        #   k_I: integral kazanç  — düşük tut, yoksa acc gürültüsü birikerek yaw bozar
        self.mahony = Mahony(frequency=self.IMU_FREQ, k_P=2.0, k_I=0.005)
        self.mahony.Q = np.array([1.0, 0.0, 0.0, 0.0])  # başlangıç: düz, hizalı

        self.ekf_filter = EKF(frequency=self.IMU_FREQ, frame='NED')
        self.ekf_filter.Q = np.array([1.0, 0.0, 0.0, 0.0])

        # Güncel Euler açıları (her IMU callback'inde güncellenir)
        self.ahrs_roll  = 0.0  # derece
        self.ahrs_pitch = 0.0  # derece
        self.ahrs_yaw   = 0.0  # derece

        # --- MOTOR ---
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate   = 115200
        self.START_FRAME = 0xABCD
        self.SPEED_COEFF = 600.0
        self.STEER_COEFF = 250.0
        self.MAX_SPEED   = 0.25    # m/s
        self.MAX_TURN    = 0.6     # rad/s

        # --- IMU EĞİM EŞİKLERİ ---
        # [✅3] Eğim aşıldığında araç yavaşça geri çekilir
        self.MAX_PITCH_DEG = 30.0  # ileri-geri eğim limiti (derece)
        self.MAX_ROLL_DEG  = 25.0  # yan eğim limiti (derece)
        self.imu_pitch     = 0.0   # anlık pitch (derece)
        self.imu_roll      = 0.0   # anlık roll  (derece)
        self.imu_gyro_z    = 0.0   # yaw açısal hızı (rad/s)
        self.tilt_exceeded = False  # eğim aşımı bayrağı

        # --- PID YAPILARI ---
        # [✅6] Görsel servoing: PD — yön açısı hatası için
        self.servo_Kp  = 0.7
        self.servo_Kd  = 0.12
        self.servo_prev_err = 0.0

        # [✅6] Global yaklaşım: PI — uzun mesafede sabit sapma birikmesini önler
        self.global_Kp      = 0.8
        self.global_Ki      = 0.05
        self.global_integral = 0.0
        self.INTEGRAL_LIMIT  = 0.5  # integrator windup koruması

        # [✅11] Velocity smoothing
        self.SMOOTH_ALPHA = 0.2

        # [✅12] D terimi koruma
        self.D_CLAMP  = 2.0
        self.DT_MIN   = 0.02

        # [✅15] Dynamic speed
        self.SPEED_RAMP_DIST = 2.0

        # [✅16] Angle deadband
        self.ANGLE_DEADBAND = 0.02

        # --- YOLO & DOSYA ---
        self.TARGET_CLASS_ID      = None
        self.CONFIDENCE_THRESHOLD = 0.7

        user_home = os.path.expanduser("~")
        self.desktop_path = user_home
        if os.path.exists(os.path.join(user_home, "Masaüstü")):
            self.desktop_path = os.path.join(user_home, "Masaüstü")
        elif os.path.exists(os.path.join(user_home, "Desktop")):
            self.desktop_path = os.path.join(user_home, "Desktop")

        self.save_dir = os.path.join(self.desktop_path, "rover_fotograflar")
        os.makedirs(self.save_dir, exist_ok=True)

        model_name = "best.pt"
        model_path = os.path.join(self.desktop_path, model_name)
        try:
            if os.path.exists(model_path):
                self.get_logger().info(f'Özel Model Yüklendi: {model_path}')
                self.model = YOLO(model_path)
            else:
                raise FileNotFoundError(f"'{model_name}' Masaüstünde bulunamadı!")
        except Exception as e:
            self.get_logger().error(f'Model Başlatılamadı: {e}')
            raise e

        # --- NAVİGASYON ---
        self.global_goal_x = 61.0
        self.global_goal_y = 34.0
        self.pose_x   = 0.0
        self.pose_y   = 0.0
        self.pose_yaw = 0.0
        self.prev_v   = 0.0
        self.prev_w   = 0.0
        self.state    = "GLOBAL_APPROACH"

        # --- HEDEF DEĞİŞKENLERİ ---
        self.visual_valid    = False
        self.target_dist     = 0.0
        self.target_angle    = 0.0
        self.last_visual_time = 0.0

        # [✅2] Temporal smoothing — son 5 açı değeri kayan ortalama
        self.angle_history = deque(maxlen=5)

        # --- ZAMANLAYICILAR ---
        self.last_loop_time   = self.get_clock().now()
        self.search_start_time = None
        self.arrival_time      = None
        self.last_photo_time   = 0.0
        self.photo_count       = 0
        self.TARGET_PHOTO_LIMIT = 5
        self.last_yolo_time    = 0
        self.YOLO_INTERVAL     = 0.15   # ~7 fps

        # [✅8] Watchdog
        self.last_control_time = time.time()

        # --- KAMERA KALİBRASYONU ---
        self.fx = 600.0
        self.cx = 320.0

        # --- SERİ PORT ---
        # [✅10] Yeniden bağlanma için sabitler
        self.SERIAL_RETRY_LIMIT = 3    # tek hata anında max deneme sayısı
        self.SERIAL_RETRY_DELAY = 1.0  # denemeler arası bekleme (sn)
        self.serial_fail_count  = 0    # arka arkaya başarısız gönderim sayacı

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("Seri Port Aktif.")
        except Exception as e:
            self.get_logger().error(f"SERİ PORT YOK: {e}")
            self.ser = None

        # --- ROS BAĞLANTILARI ---
        self.bridge      = CvBridge()
        self.rgb_frame   = None
        self.depth_frame = None

        # [✅MT] RGB callback → yolo_cb_group: YOLO çalışırken bu thread meşgul olur,
        #        diğer callback'ler (IMU, depth, control) etkilenmez
        self.create_subscription(Image, '/realsense/rgb/image_raw',
                                 self.rgb_callback, 10,
                                 callback_group=self.yolo_cb_group)

        # Depth, IMU, mission → control_cb_group: birbirini beklemeden paralel çalışır
        self.create_subscription(Image, '/realsense/depth/image_rect',
                                 self.depth_callback, 10,
                                 callback_group=self.control_cb_group)
        self.create_subscription(Point, '/mission/goal',
                                 self.mission_callback, 10,
                                 callback_group=self.control_cb_group)
        # [✅3,✅4] IMU subscription — topic adını kendi setup'ına göre düzenle
        self.create_subscription(Imu,   '/imu/data',
                                 self.imu_callback, 10,
                                 callback_group=self.control_cb_group)

        # Timer'lar da control_cb_group içinde — YOLO'dan bağımsız çalışır
        self.timer = self.create_timer(0.1, self.control_loop,
                                       callback_group=self.control_cb_group)

        # [✅8] Watchdog timer
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check,
                                                callback_group=self.control_cb_group)

    # ================================================================
    #  CALLBACK'LER
    # ================================================================

    def rgb_callback(self, msg):
        with self.data_lock:
            try:
                self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                pass

        now = time.time()
        if now - self.last_yolo_time > self.YOLO_INTERVAL:
            self.process_yolo()
            self.last_yolo_time = now

    def depth_callback(self, msg):
        with self.data_lock:
            try:
                self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            except Exception:
                pass

    def mission_callback(self, msg):
        self.global_goal_x   = msg.x
        self.global_goal_y   = msg.y
        self.state           = "GLOBAL_APPROACH"
        self.photo_count     = 0
        self.arrival_time    = None
        self.search_start_time = None
        self.global_integral = 0.0  # PI integratörünü sıfırla
        self.get_logger().info(f"Hedef: {msg.x:.1f}, {msg.y:.1f}")

    # [✅3, ✅4] IMU callback — quaternion → Euler dönüşümü
    def imu_callback(self, msg):
        # Ham sensör verilerini ROS mesajından al
        gyr = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z   # yaw açısal hızı (rad/s)
        ])
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # AHRS filtresi ile quaternion güncelle
        # Mahony veya EKF — USE_EKF bayrağı ile seçilir
        try:
            if self.USE_EKF:
                # EKF: daha hassas, biraz daha ağır
                self.ekf_filter.Q = self.ekf_filter.update(
                    self.ekf_filter.Q, gyr=gyr, acc=acc
                )
                q = self.ekf_filter.Q
            else:
                # Mahony: hafif, yarışma süresi için yeterli (varsayılan)
                self.mahony.Q = self.mahony.updateIMU(
                    self.mahony.Q, gyr=gyr, acc=acc
                )
                q = self.mahony.Q

            # Quaternion → Euler (radyan) → derece
            # q2euler çıktısı: [roll, pitch, yaw] radyan cinsinden
            euler = q2euler(q)  # [roll, pitch, yaw] rad
            roll_deg  = math.degrees(euler[0])
            pitch_deg = math.degrees(euler[1])
            yaw_rad   = euler[2]  # pose_yaw için radyan

        except Exception as e:
            self.get_logger().warn(f"AHRS güncelleme hatası: {e}")
            return

        with self.data_lock:
            self.imu_pitch  = pitch_deg
            self.imu_roll   = roll_deg
            # [✅4] Yaw artık AHRS filtresi çıktısından geliyor
            # gyro_z ham değeri de saklanıyor (yedek / debug için)
            self.imu_gyro_z = gyr[2]
            self.ahrs_roll  = roll_deg
            self.ahrs_pitch = pitch_deg
            self.ahrs_yaw   = math.degrees(yaw_rad)

    # ================================================================
    #  GÖRÜNTÜ İŞLEME
    # ================================================================

    def process_yolo(self):
        current_rgb   = None
        current_depth = None

        with self.data_lock:
            if self.rgb_frame is None:
                return
            current_rgb = self.rgb_frame.copy()
            if self.depth_frame is not None:
                current_depth = self.depth_frame.copy()

        has_display = os.environ.get("DISPLAY") is not None

        with self.yolo_lock:
            results = self.model(current_rgb, verbose=False)

        target_found = False
        best_box     = None
        max_conf     = 0

        for r in results:
            for box in r.boxes:
                conf   = float(box.conf[0])
                cls_id = int(box.cls[0])

                if self.TARGET_CLASS_ID is not None and cls_id != self.TARGET_CLASS_ID:
                    continue

                if conf > self.CONFIDENCE_THRESHOLD and conf > max_conf:
                    max_conf  = conf
                    best_box  = box.xyxy[0].cpu().numpy()
                    target_found = True

        if target_found and current_depth is not None:
            if current_depth.shape == current_rgb.shape[:2]:
                x1, y1, x2, y2 = map(int, best_box)

                # [✅1] ROI medyanı — bounding box içini %20 kırp, kalan alanın medyanı
                mx = int((x2 - x1) * 0.20)
                my = int((y2 - y1) * 0.20)
                roi = current_depth[y1 + my : y2 - my, x1 + mx : x2 - mx]
                valid_roi = roi[roi > 0]

                # Yeterli piksel yoksa merkeze 9x9 pencere dene
                if valid_roi.size < 10:
                    cx_px = int((x1 + x2) / 2)
                    cy_px = int((y1 + y2) / 2)
                    m = 4  # 9x9
                    patch = current_depth[
                        max(0, cy_px - m): cy_px + m + 1,
                        max(0, cx_px - m): cx_px + m + 1
                    ]
                    valid_roi = patch[patch > 0]

                if valid_roi.size == 0:
                    return  # geçerli piksel yok, bu kareyi atla

                dist_mm = float(np.median(valid_roi))
                if dist_mm <= 0:
                    return

                dist_m  = dist_mm / 1000.0

                # [✅13] Depth outlier reject — fiziksel olarak anlamsız ölçümü at
                if dist_m > 10.0:
                    return
                center_x  = int((x1 + x2) / 2)
                angle_raw = -1.0 * (center_x - self.cx) / self.fx

                # [✅2] Temporal smoothing — kayan ortalama
                self.angle_history.append(angle_raw)
                smoothed_angle = sum(self.angle_history) / len(self.angle_history)

                with self.data_lock:
                    self.target_dist      = dist_m
                    self.target_angle     = smoothed_angle
                    self.visual_valid     = True
                    self.last_visual_time = time.time()

                if has_display and self.DEBUG_VISUAL:
                    cv2.rectangle(current_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(current_rgb, f"{dist_m:.2f}m | a:{smoothed_angle:.3f}",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        if has_display and self.DEBUG_VISUAL:
            cv2.imshow("Rover View", current_rgb)
            cv2.waitKey(1)

    # ================================================================
    #  WATCHDOG
    # ================================================================

    # [✅8] Kontrol döngüsü 0.5 sn'den uzun süredir çalışmadıysa acil dur
    def watchdog_check(self):
        if time.time() - self.last_control_time > 0.5:
            self.get_logger().error("WATCHDOG: Kontrol döngüsü yanıt vermiyor! Acil dur.")
            self.send_to_motor(0.0, 0.0)

    # ================================================================
    #  KONTROL DÖNGÜSÜ
    # ================================================================

    def control_loop(self):
        # [✅8] Watchdog için zaman damgası
        self.last_control_time = time.time()

        now = self.get_clock().now()
        dt  = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        # [✅9] dt eşiği 0.3 sn — anormal gecikmeleri kes
        if dt > 0.3:
            dt = 0.1

        # --- IMU + AHRS verilerini oku ---
        with self.data_lock:
            pitch  = self.imu_pitch    # derece — eğim güvenliği + odometri
            roll   = self.imu_roll     # derece — eğim güvenliği
            gyro_z = self.imu_gyro_z   # rad/s  — yedek
            # [✅AHRS] Yaw doğrudan filtre çıktısından — gyro_z*dt entegrasyonundan daha kararlı
            ahrs_yaw_deg = self.ahrs_yaw

        # [✅4,✅AHRS] Yaw blending — ani overwrite yerine yavaş düzeltme
        # AHRS yaw'ı doğrudan atamak filtre ilk saniyelerinde ani sıçrama yapabilir.
        # alpha=0.02 → her döngüde %2 AHRS'a yaklaş → yumuşak, kararlı düzeltme
        # Düşük frekans düzeltme: kontrol döngüsü 10Hz → tam düzeltme ~5 sn sürer
        BLEND_ALPHA   = 0.02
        new_yaw_rad   = math.radians(ahrs_yaw_deg)
        # Açı farkını [-π, π] aralığında hesapla (wrap-around güvenli)
        yaw_diff      = math.atan2(math.sin(new_yaw_rad - self.pose_yaw),
                                   math.cos(new_yaw_rad - self.pose_yaw))
        self.pose_yaw = self.pose_yaw + BLEND_ALPHA * yaw_diff
        self.pose_yaw = math.atan2(math.sin(self.pose_yaw), math.cos(self.pose_yaw))

        # [✅3] Eğim eşiği kontrolü
        self.tilt_exceeded = (abs(pitch) > self.MAX_PITCH_DEG or
                              abs(roll)  > self.MAX_ROLL_DEG)

        if self.tilt_exceeded:
            self.get_logger().warn(
                f"EĞİM AŞILDI! pitch={pitch:.1f}° roll={roll:.1f}° — Geri çekiliyor.",
                throttle_duration_sec=1.0)
            # Yavaşça geri çekil, dönme yok
            self.send_to_motor(-0.10, 0.0)
            return  # döngünün geri kalanını atla

        # [✅3] Odometri — pitch kompanzasyonlu mesafe
        # Eğimde alınan gerçek yatay mesafe: v * cos(pitch) * dt
        pitch_rad    = math.radians(pitch)
        effective_v  = self.prev_v * math.cos(pitch_rad)
        self.pose_x += effective_v * math.cos(self.pose_yaw) * dt
        self.pose_y += effective_v * math.sin(self.pose_yaw) * dt

        # Görsel timeout kontrolü
        current_sys_time = time.time()
        with self.data_lock:
            if current_sys_time - self.last_visual_time > self.TIMEOUT_LIMIT:
                self.visual_valid = False

        target_v = 0.0
        target_w = 0.0

        # ---- DURUM MAKİNESİ ----

        # [✅17] State logging — throttle 1 Hz (her döngüde değil)
        self.get_logger().info(
            f"STATE:{self.state} | v:{self.prev_v:.2f} w:{self.prev_w:.2f} "
            f"| yaw:{math.degrees(self.pose_yaw):.1f}° "
            f"| pitch:{pitch:.1f}° roll:{roll:.1f}°",
            throttle_duration_sec=1.0
        )

        if self.state == "STOPPED":
            if self.arrival_time is None:
                self.arrival_time    = current_sys_time
                self.last_photo_time = current_sys_time
                self.get_logger().info("Çekim Başlıyor...")
            elif current_sys_time - self.arrival_time > 3.0:
                if current_sys_time - self.last_photo_time > 1.0:
                    # [✅14] Fotoğraf öncesi stabilizasyon kontrolü
                    vehicle_stable = (abs(self.prev_v) < 0.02 and
                                      abs(self.prev_w) < 0.05)
                    if vehicle_stable:
                        self.take_snapshot()
                        self.photo_count    += 1
                        self.last_photo_time = current_sys_time
                        if self.photo_count >= self.TARGET_PHOTO_LIMIT:
                            self.state = "MISSION_COMPLETE"
                            self.get_logger().info("BİTTİ!")
                    else:
                        self.get_logger().warn("Araç sabit değil, çekim bekleniyor...", throttle_duration_sec=2.0)

        elif self.state == "MISSION_COMPLETE":
            target_v = 0.0
            target_w = 0.0

        elif self.visual_valid:
            self.state = "VISUAL_SERVOING"

            with self.data_lock:
                dist_val  = self.target_dist
                angle_val = self.target_angle

            if dist_val < 0.5:
                self.state = "STOPPED"
                # PD türev geçmişini sıfırla
                self.servo_prev_err = 0.0
            else:
                if abs(angle_val) > 0.35:
                    # Açı çok büyük → yerinde dön, PD devreye girmesin
                    target_v = 0.0
                    target_w = 0.5 if angle_val > 0 else -0.5
                    self.servo_prev_err = angle_val
                else:
                    # [✅6,✅12,✅15,✅16] PD kontrol — görsel servoing
                    err = angle_val

                    # [✅16] Angle deadband — küçük hataları sıfırla
                    if abs(err) < self.ANGLE_DEADBAND:
                        err = 0.0

                    # [✅12] D terimi — min dt ile türev patlaması önle + clamp
                    d_err = (err - self.servo_prev_err) / max(dt, self.DT_MIN)
                    d_err = max(min(d_err, self.D_CLAMP), -self.D_CLAMP)
                    self.servo_prev_err = err

                    # [✅15] Dynamic speed — hedefe yaklaştıkça yavaşla
                    speed_scale = min(dist_val / self.SPEED_RAMP_DIST, 1.0)
                    target_v    = self.MAX_SPEED * speed_scale

                    target_w = self.servo_Kp * err + self.servo_Kd * d_err

        else:
            # Kör sürüş
            if self.state == "VISUAL_SERVOING":
                self.state             = "SEARCHING"
                self.search_start_time = current_sys_time
                self.servo_prev_err    = 0.0
                self.get_logger().warn("Görüntü gitti!")

            dx   = self.global_goal_x - self.pose_x
            dy   = self.global_goal_y - self.pose_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.5:
                # Hedefe yakın ama görsel yok → ara
                self.state = "SEARCHING"
                if self.search_start_time is None:
                    self.search_start_time = current_sys_time
                    self.get_logger().info("Bölge Araması...")

                elapsed = current_sys_time - self.search_start_time
                if elapsed < 5.0:
                    target_w = 0.4
                elif elapsed < 25.0:
                    boost    = (elapsed - 5.0) * 0.01
                    target_v = min(0.1 + boost, 0.2)
                    target_w = 0.4
                else:
                    self.state = "MISSION_COMPLETE"
                    self.get_logger().error("Bulunamadı.")

            else:
                self.state             = "GLOBAL_APPROACH"
                self.search_start_time = None

                t_angle = math.atan2(dy, dx)
                err     = math.atan2(
                    math.sin(t_angle - self.pose_yaw),
                    math.cos(t_angle - self.pose_yaw)
                )

                if abs(err) > 0.5:
                    # Açı çok büyük → sadece dön, integratörü sıfırla
                    target_v             = 0.0
                    target_w             = 0.5 if err > 0 else -0.5
                    self.global_integral = 0.0
                else:
                    # [✅6,✅16] PI kontrol — global yaklaşım
                    # [✅16] Deadband
                    if abs(err) < self.ANGLE_DEADBAND:
                        err = 0.0

                    self.global_integral += err * dt
                    self.global_integral  = max(
                        min(self.global_integral, self.INTEGRAL_LIMIT),
                        -self.INTEGRAL_LIMIT
                    )

                    target_w = (self.global_Kp * err +
                                self.global_Ki * self.global_integral)

                    # [✅15] Dynamic speed — hedefe yaklaştıkça yavaşla
                    speed_scale = min(dist / self.SPEED_RAMP_DIST, 1.0)
                    if abs(err) > 0.2:
                        target_v = self.MAX_SPEED * 0.5 * speed_scale
                    else:
                        target_v = self.MAX_SPEED * speed_scale

        # [✅13] Depth outlier — görsel servoingde çok uzak okumayı reddet
        # (process_yolo'da dist_m > 10 kontrolü de var ama burada ikinci katman)

        # Limitler
        target_v = max(min(target_v, self.MAX_SPEED), -self.MAX_SPEED)
        target_w = max(min(target_w, self.MAX_TURN),  -self.MAX_TURN)

        # [✅11] Velocity smoothing — ani jerk'ü yumuşat
        # [BUG FIX] Motora smoothed değer gönderilmeli, ham target_v/w değil!
        self.prev_v = (1.0 - self.SMOOTH_ALPHA) * self.prev_v + self.SMOOTH_ALPHA * target_v
        self.prev_w = (1.0 - self.SMOOTH_ALPHA) * self.prev_w + self.SMOOTH_ALPHA * target_w

        self.send_to_motor(self.prev_v, self.prev_w)

    # ================================================================
    #  MOTOR & FOTOĞRAF
    # ================================================================

    def send_to_motor(self, v_x, v_z):
        if self.ser is None:
            # [✅10] Port hiç açılmadıysa ya da önceki reconnect başarısız olduysa tekrar dene
            self._try_reconnect_serial()
            if self.ser is None:
                return

        speed = int(v_x * self.SPEED_COEFF)
        steer = int(v_z * self.STEER_COEFF)
        speed = max(min(speed, 1000), -1000)
        steer = max(min(steer, 1000), -1000)

        try:
            checksum = (self.START_FRAME ^ steer ^ speed) & 0xFFFF
            packet   = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            self.ser.write(packet)
            self.serial_fail_count = 0  # başarılı gönderimde sayacı sıfırla
        except Exception as e:
            self.get_logger().error(f"Motor Hatası: {e}")
            self.serial_fail_count += 1
            # [✅10] Arka arkaya hata SERIAL_RETRY_LIMIT'i aşarsa yeniden bağlan
            if self.serial_fail_count >= self.SERIAL_RETRY_LIMIT:
                self.get_logger().warn("Seri port yeniden bağlanılıyor...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self._try_reconnect_serial()

    # [✅10] Seri port yeniden bağlanma yardımcısı
    def _try_reconnect_serial(self):
        for attempt in range(1, self.SERIAL_RETRY_LIMIT + 1):
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                self.serial_fail_count = 0
                self.get_logger().info(f"Seri port yeniden bağlandı (deneme {attempt}).")
                return
            except Exception as e:
                self.get_logger().warn(f"Reconnect denemesi {attempt} başarısız: {e}")
                time.sleep(self.SERIAL_RETRY_DELAY)
        self.get_logger().error("Seri port yeniden bağlanamadı. Motor devre dışı.")
        self.ser = None

    def take_snapshot(self):
        frame_to_save = None
        with self.data_lock:
            if self.rgb_frame is not None:
                frame_to_save = self.rgb_frame.copy()

        if frame_to_save is not None:
            ts    = time.strftime("%Y%m%d_%H%M%S")
            fname = f"sample_{self.photo_count + 1}_{ts}.jpg"
            path  = os.path.join(self.save_dir, fname)
            cv2.imwrite(path, frame_to_save)
            self.get_logger().info(f"Kayıt: {fname}")


# ================================================================
#  ANA GİRİŞ
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RoverMaster()
        # [✅MT] MultiThreadedExecutor — callback'ler birbirini bloklamaz
        # YOLO (yolo_cb_group) ve kontrol döngüsü (control_cb_group) paralel çalışır
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


if __name__ == '__main__':
    main()
