#!/usr/bin/env python3
"""
================================================================================
              INTEL REALSENSE D435i DEPTH KAMERA ROS 2 PUBLISHER NODE
================================================================================

GENEL BAKIŞ:
------------
Bu modül, Intel RealSense D435i derinlik kamerasından tüm sensör verilerini
(RGB, Depth, Infrared, IMU) alarak ROS 2 ekosisteminde topic'ler aracılığıyla
yayınlayan bir publisher node implementasyonudur. 3D haritalama, SLAM, nesne
tespiti, engel algılama ve robotik navigasyon uygulamaları için temel veri
kaynağı sağlar.

D435i KAMERA ÖZELLİKLERİ:
-------------------------
┌─────────────────────────────────────────────────────────────────────────────┐
│ Sensör            │ Çözünürlük      │ FPS    │ FOV (HxV)    │ Açıklama      │
├───────────────────┼─────────────────┼────────┼──────────────┼───────────────┤
│ RGB Kamera        │ 1920x1080 max   │ 30     │ 69°x42°      │ Renkli görüntü│
│ Depth Sensör      │ 1280x720 max    │ 90     │ 87°x58°      │ Stereo IR     │
│ IR Kamera (Sol)   │ 1280x720 max    │ 90     │ 87°x58°      │ Kızılötesi    │
│ IR Kamera (Sağ)   │ 1280x720 max    │ 90     │ 87°x58°      │ Kızılötesi    │
│ IMU (Accel+Gyro)  │ -               │ 250/400│ -            │ 6-DOF hareket │
└─────────────────────────────────────────────────────────────────────────────┘
* Depth menzil: 0.1m - 10m (optimal: 0.3m - 3m)
* Baseline (IR sensör arası mesafe): 50mm

MİMARİ YAPI:
------------
┌─────────────────────┐     ┌──────────────────────────┐     ┌──────────────────┐
│  Intel RealSense    │     │   RealSensePublisher     │     │   ROS 2 Topics   │
│  D435i Kamera       │────▶│   Node                   │────▶│                  │
│                     │     │   (pyrealsense2 +        │     │  /realsense/*    │
│  ┌───────────────┐  │     │    cv_bridge)            │     │                  │
│  │ RGB Sensor    │  │     │                          │     │  ├─ rgb/         │
│  │ Depth Sensor  │  │     │  ┌────────────────────┐  │     │  ├─ depth/       │
│  │ IR Left       │  │     │  │ rs.pipeline()      │  │     │  ├─ infra1/      │
│  │ IR Right      │  │     │  │ rs.align()         │  │     │  ├─ infra2/      │
│  │ IMU (6-DOF)   │  │     │  │ rs.config()        │  │     │  └─ imu/         │
│  └───────────────┘  │     │  └────────────────────┘  │     │                  │
└─────────────────────┘     └──────────────────────────┘     └──────────────────┘

ÇALIŞMA PRENSİBİ:
-----------------
1. BAŞLATMA (Initialization):
   - Node parametreleri okunur (çözünürlük, FPS, IMU aktif/pasif, vb.)
   - ROS 2 publisher'ları oluşturulur (her stream için ayrı topic)
   - pyrealsense2 pipeline ve config yapılandırılır
   - Cihaz seri numarasıyla tanınır ve bağlanır
   - Stream'ler (RGB, Depth, IR, IMU) aktifleştirilir
   - Alignment objesi oluşturulur (depth-to-color hizalama)

2. STREAM BAŞLATMA SIRASI:
   ┌──────────────────────────────────────────────────────────────┐
   │  rs.context() ──▶ query_devices() ──▶ enable_device(serial) │
   │       │                                                      │
   │       ▼                                                      │
   │  rs.config() ──▶ enable_stream(color) ──┐                   │
   │       │         enable_stream(depth) ───┤                   │
   │       │         enable_stream(infrared)─┤                   │
   │       │         enable_stream(accel) ───┤                   │
   │       │         enable_stream(gyro) ────┘                   │
   │       ▼                                                      │
   │  rs.pipeline.start(config) ──▶ rs.align(color)              │
   └──────────────────────────────────────────────────────────────┘

3. FRAME DÖNGÜSÜ (Main Loop):
   - Timer callback'i belirlenen FPS'e göre tetiklenir (varsayılan 30 Hz)
   - pipeline.wait_for_frames() ile senkronize frame set alınır
   - Her frame tipi ayrı ayrı işlenir ve ilgili topic'e publish edilir
   - IMU verileri asenkron olarak okunur ve publish edilir

4. DEPTH ALIGNMENT (Hizalama):
   ┌─────────────────────────────────────────────────────────────────────────┐
   │                        DEPTH-TO-COLOR ALIGNMENT                        │
   │                                                                         │
   │   Depth Frame          Aligned Depth           RGB Frame               │
   │   (Depth FOV)          (RGB FOV)               (RGB FOV)               │
   │  ┌───────────┐        ┌───────────┐          ┌───────────┐            │
   │  │           │        │           │          │           │            │
   │  │  ░░░░░░░  │  ───▶  │  ░░░░░░░  │    +     │  ▓▓▓▓▓▓▓  │            │
   │  │  ░░░░░░░  │ align  │  ░░░░░░░  │          │  ▓▓▓▓▓▓▓  │            │
   │  │           │        │           │          │           │            │
   │  └───────────┘        └───────────┘          └───────────┘            │
   │                              │                      │                  │
   │                              └──────────┬───────────┘                  │
   │                                         ▼                              │
   │                                  RGBD Point Cloud                      │
   │                                  (Her pikselde RGB + Z)                │
   └─────────────────────────────────────────────────────────────────────────┘

YAYIN AKIŞI (Publishing Pipeline):
----------------------------------
   [RealSense Frameset]
          │
          ├──▶ [Color Frame] ──────────────────▶ /realsense/rgb/image_raw
          │         └──▶ [Intrinsics] ─────────▶ /realsense/rgb/camera_info
          │
          ├──▶ [Depth Frame] ──────────────────▶ /realsense/depth/image_rect
          │         └──▶ [Intrinsics] ─────────▶ /realsense/depth/camera_info
          │
          ├──▶ [Aligned Depth] ────────────────▶ /realsense/depth/color_aligned
          │         (RGB FOV'una hizalanmış)
          │
          ├──▶ [IR Frame 1] ───────────────────▶ /realsense/infra1/image_rect
          │
          ├──▶ [IR Frame 2] ───────────────────▶ /realsense/infra2/image_rect
          │
          ├──▶ [Accelerometer] ────────────────▶ /realsense/imu/accel
          │
          └──▶ [Gyroscope] ────────────────────▶ /realsense/imu/gyro

YAYINLANAN TOPIC'LER:
---------------------
┌─────────────────────────────────┬─────────────────┬─────────────────────────┐
│ Topic Adı                       │ Mesaj Tipi      │ Açıklama                │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/rgb/image_raw        │ sensor_msgs/    │ Renkli görüntü (BGR8)   │
│                                 │ Image           │ 848x480 @ 30 FPS        │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/rgb/camera_info      │ sensor_msgs/    │ RGB kamera intrinsics   │
│                                 │ CameraInfo      │ K, D, R, P matrisleri   │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/depth/image_rect     │ sensor_msgs/    │ Derinlik haritası       │
│                                 │ Image           │ 16-bit (mm), 16UC1      │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/depth/camera_info    │ sensor_msgs/    │ Depth kamera intrinsics │
│                                 │ CameraInfo      │ Stereo kalibrasyon      │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/depth/color_aligned  │ sensor_msgs/    │ RGB'ye hizalanmış depth │
│                                 │ Image           │ Piksel-piksel eşleşme   │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/infra1/image_rect    │ sensor_msgs/    │ Sol IR kamera (mono8)   │
│                                 │ Image           │ Stereo matching için    │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/infra2/image_rect    │ sensor_msgs/    │ Sağ IR kamera (mono8)   │
│                                 │ Image           │ Stereo matching için    │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/imu/accel            │ sensor_msgs/    │ Lineer ivme (m/s²)      │
│                                 │ Imu             │ x, y, z eksenleri       │
├─────────────────────────────────┼─────────────────┼─────────────────────────┤
│ /realsense/imu/gyro             │ sensor_msgs/    │ Açısal hız (rad/s)      │
│                                 │ Imu             │ x, y, z eksenleri       │
└─────────────────────────────────┴─────────────────┴─────────────────────────┘

DEPTH VERİ FORMATI:
-------------------
┌─────────────────────────────────────────────────────────────────────────────┐
│  Encoding: 16UC1 (16-bit unsigned, single channel)                         │
│  Birim: Milimetre (mm)                                                     │
│  Değer Aralığı: 0 - 65535 (0 = geçersiz/ölçülemedi)                        │
│                                                                             │
│  Örnek dönüşüm:                                                            │
│    depth_value = 1500  →  Gerçek mesafe = 1.5 metre                        │
│    depth_value = 0     →  Geçersiz ölçüm (çok yakın/uzak/yansıtıcı yüzey)  │
│                                                                             │
│  Python'da metre cinsine çevirme:                                          │
│    depth_meters = depth_image.astype(np.float32) / 1000.0                  │
└─────────────────────────────────────────────────────────────────────────────┘

IMU KOORDİNAT SİSTEMİ:
----------------------
            Z (yukarı)
            │
            │
            │
            └───────── Y (sağa)
           ╱
          ╱
         X (ileri - kamera bakış yönü)

Accelerometer (İvmeölçer):
  - Durağan halde: accel.z ≈ -9.81 m/s² (yerçekimi)
  - Birim: m/s²

Gyroscope (Jiroskop):
  - Birim: rad/s
  - Pozitif yön: Sağ el kuralı (eksen etrafında saat yönünün tersi)

KONFİGÜRASYON PARAMETRELERİ:
----------------------------
Parametre              Varsayılan   Açıklama
─────────────────────────────────────────────────────────────────────────────
rgb_width              848          RGB görüntü genişliği (piksel)
rgb_height             480          RGB görüntü yüksekliği (piksel)
depth_width            848          Depth görüntü genişliği (piksel)
depth_height           480          Depth görüntü yüksekliği (piksel)
fps                    30           Hedef frame rate (Hz)
enable_imu             True         IMU verisi yayını aktif/pasif
enable_infrared        True         IR kamera yayını aktif/pasif
align_depth_to_color   True         Depth-RGB hizalama aktif/pasif

DESTEKLENEN ÇÖZÜNÜRLÜKLER:
--------------------------
RGB (Color):
  - 1920x1080 @ 30 FPS
  - 1280x720 @ 30 FPS
  - 848x480 @ 30/60 FPS (önerilen)
  - 640x480 @ 30/60 FPS

Depth/IR:
  - 1280x720 @ 30 FPS
  - 848x480 @ 30/60/90 FPS (önerilen)
  - 640x480 @ 30/60/90 FPS
  - 480x270 @ 30/60/90 FPS

KULLANIM ÖRNEKLERİ:
-------------------
# Varsayılan parametrelerle başlatma:
$ ros2 run <paket_adi> realsense_publisher

# Yüksek çözünürlük modunda başlatma:
$ ros2 run <paket_adi> realsense_publisher --ros-args \
    -p rgb_width:=1280 \
    -p rgb_height:=720 \
    -p depth_width:=1280 \
    -p depth_height:=720 \
    -p fps:=30

# Sadece RGB ve Depth (IMU/IR kapalı):
$ ros2 run <paket_adi> realsense_publisher --ros-args \
    -p enable_imu:=false \
    -p enable_infrared:=false

# Topic'leri görüntüleme:
$ ros2 topic list | grep realsense
$ ros2 topic hz /realsense/rgb/image_raw
$ ros2 topic hz /realsense/depth/image_rect

# Depth görüntüsünü RViz2'de görüntüleme:
$ rviz2
# Add → By topic → /realsense/depth/image_rect

# Point cloud oluşturma (depth_image_proc ile):
$ ros2 launch depth_image_proc point_cloud_xyz.launch.py \
    depth_image_topic:=/realsense/depth/color_aligned \
    camera_info_topic:=/realsense/rgb/camera_info

BAĞIMLILIKLAR:
--------------
- rclpy             : ROS 2 Python client library
- sensor_msgs       : Image, CameraInfo, Imu mesaj tipleri
- cv_bridge         : OpenCV <-> ROS mesaj dönüşümü
- pyrealsense2      : Intel RealSense SDK Python bindings
- numpy             : Sayısal hesaplamalar

PYREALSENSE2 KURULUMU:
----------------------
# pip ile kurulum:
$ pip install pyrealsense2

# veya kaynak koddan:
$ git clone https://github.com/IntelRealSense/librealsense
$ cd librealsense
$ mkdir build && cd build
$ cmake .. -DBUILD_PYTHON_BINDINGS=ON
$ make -j4 && sudo make install

SINIF HİYERARŞİSİ:
------------------
rclpy.node.Node
       │
       └── RealSensePublisher
              │
              ├── __init__()                 : Parametre ve publisher başlatma
              ├── create_publishers()        : Topic publisher'ları oluşturma
              ├── initialize_realsense()     : Pipeline ve stream yapılandırma
              ├── publish_frames()           : Ana timer callback fonksiyonu
              ├── publish_rgb_frame()        : BGR8 RGB görüntü yayını
              ├── publish_depth_frame()      : 16UC1 depth görüntü yayını
              ├── publish_aligned_depth_frame() : Hizalanmış depth yayını
              ├── publish_infrared_frame()   : IR görüntü yayını (sol/sağ)
              ├── publish_imu_accel()        : İvmeölçer verisi yayını
              ├── publish_imu_gyro()         : Jiroskop verisi yayını
              ├── create_camera_info()       : Kalibrasyon mesajı oluşturma
              ├── print_stats()              : Frame istatistikleri
              └── destroy_node()             : Temizlik ve pipeline durdurma

FRAME SENKRONİZASYONU:
----------------------
┌─────────────────────────────────────────────────────────────────────────────┐
│  pipeline.wait_for_frames() metodu tüm aktif stream'lerden                 │
│  senkronize frame set döndürür.                                            │
│                                                                             │
│  Frameset:                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐           │
│  │ Timestamp: T                                                 │           │
│  │ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ │           │
│  │ │ Color   │ │ Depth   │ │ IR Left │ │ IR Right│ │ IMU     │ │           │
│  │ │ Frame   │ │ Frame   │ │ Frame   │ │ Frame   │ │ Data    │ │           │
│  │ └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘ │           │
│  └─────────────────────────────────────────────────────────────┘           │
│                                                                             │
│  NOT: IMU farklı frekansta çalışır (250-400 Hz), interpolasyon gerekebilir │
└─────────────────────────────────────────────────────────────────────────────┘

PERFORMANS NOTLARI:
-------------------
- USB 3.0 bağlantı zorunludur (USB 2.0'da düşük FPS ve hata)
- 848x480 @ 30 FPS tipik CPU kullanımı: ~10-15%
- Alignment işlemi ek CPU yükü getirir (~5%)
- Birden fazla stream aktifken USB bant genişliği kritik
- Depth quality presets: High Accuracy / High Density / Medium Density

D435 vs D435i FARKI:
--------------------
┌───────────────┬─────────────┬─────────────┐
│ Özellik       │ D435        │ D435i       │
├───────────────┼─────────────┼─────────────┤
│ RGB Kamera    │ ✓           │ ✓           │
│ Depth Sensör  │ ✓           │ ✓           │
│ IR Kameralar  │ ✓           │ ✓           │
│ IMU (6-DOF)   │ ✗           │ ✓           │
│ VIO Desteği   │ ✗           │ ✓           │
└───────────────┴─────────────┴─────────────┘
* Bu kod D435'te de çalışır, sadece IMU özellikleri devre dışı kalır

HATA AYIKLAMA:
--------------
- "No RealSense device found": 
  → USB bağlantısını kontrol edin
  → rs-enumerate-devices komutu ile cihazı görün
  → udev rules kurulumu: /etc/udev/rules.d/99-realsense-libusb.rules

- Düşük FPS:
  → USB 3.0 port kullandığınızdan emin olun
  → Çözünürlüğü düşürün
  → Infrared stream'leri kapatın

- Depth görüntüsünde boşluklar:
  → Yansıtıcı yüzeyler veya güneş ışığı etkisi
  → Depth filtreleri uygulayın (temporal, spatial, hole-filling)

- IMU verisi gelmiyor:
  → D435 modelinde IMU yoktur (D435i gerekli)
  → enable_imu parametresini kontrol edin

REALSENSE-ROS KARŞILAŞTIRMASI:
------------------------------
Bu custom node vs. Intel'in resmi realsense-ros paketi:

Custom Node (Bu kod):
  ✓ Hafif ve minimal bağımlılık
  ✓ Kolay özelleştirme
  ✓ Öğrenme amaçlı ideal
  ✗ Tüm RealSense özellikleri yok

realsense-ros (Intel resmi):
  ✓ Tüm özellikler ve filtreler
  ✓ Dinamik rekonfigürasyon
  ✓ Launch dosyaları hazır
  ✗ Daha karmaşık bağımlılık ağacı

İLGİLİ ROS 2 PAKETLERİ:
-----------------------
- depth_image_proc  : Depth görüntü işleme, point cloud oluşturma
- image_pipeline    : Görüntü işleme araçları
- rtabmap_ros       : RGB-D SLAM
- octomap           : 3D occupancy mapping
- pointcloud_to_laserscan : 3D→2D dönüşüm (navigasyon için)

YAZAR: [Proje Sahibi]
TARİH: 2024
LİSANS: [Lisans Bilgisi]
VERSİYON: 1.0.0
================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, Temperature
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time


class RealSensePublisher(Node):
    """
    Intel RealSense D435i kamerasından tüm stream'leri ROS 2'ye yayınlar
    
    Yayınlanan Topic'ler:
    - /realsense/rgb/image_raw          : RGB görüntü (848x480 @ 30 FPS)
    - /realsense/rgb/camera_info        : RGB kamera kalibrasyon bilgisi
    - /realsense/depth/image_rect       : Depth görüntü (848x480 @ 30 FPS)
    - /realsense/depth/camera_info      : Depth kamera kalibrasyon bilgisi
    - /realsense/depth/color_aligned    : RGB'ye hizalanmış depth
    - /realsense/infra1/image_rect      : Sol kızılötesi kamera
    - /realsense/infra2/image_rect      : Sağ kızılötesi kamera
    - /realsense/imu/accel              : İvmeölçer verisi (D435i'de var)
    - /realsense/imu/gyro               : Jiroskop verisi (D435i'de var)
    """
    
    def __init__(self):
        super().__init__('realsense_publisher_node')
        
        # Parametreler
        self.declare_parameter('rgb_width', 848)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('depth_width', 848)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_infrared', True)
        self.declare_parameter('align_depth_to_color', True)
        
        self.rgb_width = self.get_parameter('rgb_width').value
        self.rgb_height = self.get_parameter('rgb_height').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.enable_infrared = self.get_parameter('enable_infrared').value
        self.align_depth = self.get_parameter('align_depth_to_color').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # RealSense pipeline ve config
        self.pipeline = None
        self.config = None
        self.align = None
        
        # Publishers oluştur
        self.create_publishers()
        
        # RealSense'i başlat
        self.initialize_realsense()
        
        # Timer ile frame'leri publish et
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frames)
        
        # İstatistikler
        self.frame_count = {
            'rgb': 0,
            'depth': 0,
            'infra1': 0,
            'infra2': 0,
            'aligned_depth': 0,
            'imu_accel': 0,
            'imu_gyro': 0
        }
        
        self.get_logger().info('🎥 RealSense Publisher Node başlatıldı')
        self.get_logger().info(f'📸 RGB: {self.rgb_width}x{self.rgb_height} @ {self.fps} FPS')
        self.get_logger().info(f'📏 Depth: {self.depth_width}x{self.depth_height} @ {self.fps} FPS')
        self.get_logger().info(f'🔧 IMU: {"Aktif" if self.enable_imu else "Pasif"}')
        self.get_logger().info(f'🔧 Infrared: {"Aktif" if self.enable_infrared else "Pasif"}')
        self.get_logger().info(f'🔧 Aligned Depth: {"Aktif" if self.align_depth else "Pasif"}')
    
    def create_publishers(self):
        """Tüm ROS publisher'ları oluştur"""
        
        # RGB görüntü ve camera info
        self.pub_rgb = self.create_publisher(
            Image, '/realsense/rgb/image_raw', 10)
        self.pub_rgb_info = self.create_publisher(
            CameraInfo, '/realsense/rgb/camera_info', 10)
        
        # Depth görüntü ve camera info
        self.pub_depth = self.create_publisher(
            Image, '/realsense/depth/image_rect', 10)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, '/realsense/depth/camera_info', 10)
        
        # RGB'ye hizalanmış depth
        if self.align_depth:
            self.pub_aligned_depth = self.create_publisher(
                Image, '/realsense/depth/color_aligned', 10)
        
        # Infrared kameralar
        if self.enable_infrared:
            self.pub_infra1 = self.create_publisher(
                Image, '/realsense/infra1/image_rect', 10)
            self.pub_infra2 = self.create_publisher(
                Image, '/realsense/infra2/image_rect', 10)
        
        # IMU verileri (D435i'de mevcut)
        if self.enable_imu:
            self.pub_imu_accel = self.create_publisher(
                Imu, '/realsense/imu/accel', 10)
            self.pub_imu_gyro = self.create_publisher(
                Imu, '/realsense/imu/gyro', 10)
        
        self.get_logger().info('✅ Tüm publisher\'lar oluşturuldu')
    
    def initialize_realsense(self):
        """RealSense kamerasını başlat ve yapılandır"""
        try:
            # Pipeline ve config oluştur
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Cihazı bul
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().error('❌ RealSense cihazı bulunamadı!')
                raise RuntimeError('No RealSense device found')
            
            device = devices[0]
            device_name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)
            self.get_logger().info(f'📷 Cihaz bulundu: {device_name} (S/N: {serial})')
            
            # Serial number ile cihazı seç
            self.config.enable_device(serial)
            
            # RGB stream
            self.config.enable_stream(
                rs.stream.color,
                self.rgb_width,
                self.rgb_height,
                rs.format.bgr8,
                self.fps
            )
            
            # Depth stream
            self.config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.fps
            )
            
            # Infrared streams
            if self.enable_infrared:
                self.config.enable_stream(
                    rs.stream.infrared, 1,
                    self.depth_width, self.depth_height,
                    rs.format.y8, self.fps
                )
                self.config.enable_stream(
                    rs.stream.infrared, 2,
                    self.depth_width, self.depth_height,
                    rs.format.y8, self.fps
                )
            
            # IMU streams (D435i için)
            if self.enable_imu:
                try:
                    self.config.enable_stream(rs.stream.accel)
                    self.config.enable_stream(rs.stream.gyro)
                    self.get_logger().info('✅ IMU stream\'leri aktifleştirildi')
                except Exception as e:
                    self.get_logger().warn(f'⚠️ IMU başlatılamadı (D435 modelinde IMU yok): {e}')
                    self.enable_imu = False
            
            # Pipeline'ı başlat
            profile = self.pipeline.start(self.config)
            
            # Align objesi oluştur (depth'i RGB'ye hizalamak için)
            if self.align_depth:
                align_to = rs.stream.color
                self.align = rs.align(align_to)
            
            # Birkaç frame atla (otomatik exposure için)
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            self.get_logger().info('✅ RealSense başarıyla başlatıldı')
            
        except Exception as e:
            self.get_logger().error(f'❌ RealSense başlatma hatası: {e}')
            raise
    
    def publish_frames(self):
        """Her timer tick'inde frame'leri al ve publish et"""
        try:
            # Frame set al
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            
            # Timestamp oluştur
            timestamp = self.get_clock().now().to_msg()
            
            # RGB frame
            color_frame = frames.get_color_frame()
            if color_frame:
                self.publish_rgb_frame(color_frame, timestamp)
            
            # Depth frame
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                self.publish_depth_frame(depth_frame, timestamp)
            
            # Aligned depth
            if self.align_depth and color_frame and depth_frame:
                aligned_frames = self.align.process(frames)
                aligned_depth = aligned_frames.get_depth_frame()
                if aligned_depth:
                    self.publish_aligned_depth_frame(aligned_depth, timestamp)
            
            # Infrared frames
            if self.enable_infrared:
                infra1 = frames.get_infrared_frame(1)
                infra2 = frames.get_infrared_frame(2)
                if infra1:
                    self.publish_infrared_frame(infra1, timestamp, 1)
                if infra2:
                    self.publish_infrared_frame(infra2, timestamp, 2)
            
            # IMU verileri
            if self.enable_imu:
                # Accel
                accel_frame = frames.first_or_default(rs.stream.accel)
                if accel_frame:
                    self.publish_imu_accel(accel_frame, timestamp)
                
                # Gyro
                gyro_frame = frames.first_or_default(rs.stream.gyro)
                if gyro_frame:
                    self.publish_imu_gyro(gyro_frame, timestamp)
        
        except Exception as e:
            self.get_logger().error(f'❌ Frame publish hatası: {e}')
    
    def publish_rgb_frame(self, frame, timestamp):
        """RGB frame'i ROS topic'e publish et"""
        # Frame'i numpy array'e çevir
        rgb_image = np.asanyarray(frame.get_data())
        
        # ROS Image mesajı oluştur
        msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_color_optical_frame'
        
        # Publish et
        self.pub_rgb.publish(msg)
        
        # Camera info publish et
        rgb_info = self.create_camera_info(frame, timestamp, 'realsense_color_optical_frame')
        self.pub_rgb_info.publish(rgb_info)
        
        self.frame_count['rgb'] += 1
    
    def publish_depth_frame(self, frame, timestamp):
        """Depth frame'i ROS topic'e publish et"""
        # Frame'i numpy array'e çevir (16-bit)
        depth_image = np.asanyarray(frame.get_data())
        
        # ROS Image mesajı oluştur
        msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_depth_optical_frame'
        
        # Publish et
        self.pub_depth.publish(msg)
        
        # Camera info publish et
        depth_info = self.create_camera_info(frame, timestamp, 'realsense_depth_optical_frame')
        self.pub_depth_info.publish(depth_info)
        
        self.frame_count['depth'] += 1
    
    def publish_aligned_depth_frame(self, frame, timestamp):
        """RGB'ye hizalanmış depth frame'i publish et"""
        depth_image = np.asanyarray(frame.get_data())
        
        msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_color_optical_frame'
        
        self.pub_aligned_depth.publish(msg)
        self.frame_count['aligned_depth'] += 1
    
    def publish_infrared_frame(self, frame, timestamp, camera_num):
        """Infrared frame'i publish et"""
        ir_image = np.asanyarray(frame.get_data())
        
        msg = self.bridge.cv2_to_imgmsg(ir_image, encoding='mono8')
        msg.header.stamp = timestamp
        msg.header.frame_id = f'realsense_infra{camera_num}_optical_frame'
        
        if camera_num == 1:
            self.pub_infra1.publish(msg)
            self.frame_count['infra1'] += 1
        else:
            self.pub_infra2.publish(msg)
            self.frame_count['infra2'] += 1
    
    def publish_imu_accel(self, frame, timestamp):
        """IMU ivmeölçer verisini publish et"""
        accel_data = frame.as_motion_frame().get_motion_data()
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_imu_optical_frame'
        
        msg.linear_acceleration.x = float(accel_data.x)
        msg.linear_acceleration.y = float(accel_data.y)
        msg.linear_acceleration.z = float(accel_data.z)
        
        self.pub_imu_accel.publish(msg)
        self.frame_count['imu_accel'] += 1
    
    def publish_imu_gyro(self, frame, timestamp):
        """IMU jiroskop verisini publish et"""
        gyro_data = frame.as_motion_frame().get_motion_data()
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'realsense_imu_optical_frame'
        
        msg.angular_velocity.x = float(gyro_data.x)
        msg.angular_velocity.y = float(gyro_data.y)
        msg.angular_velocity.z = float(gyro_data.z)
        
        self.pub_imu_gyro.publish(msg)
        self.frame_count['imu_gyro'] += 1
    
    def create_camera_info(self, frame, timestamp, frame_id):
        """Camera calibration bilgisini oluştur"""
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        msg.width = intrinsics.width
        msg.height = intrinsics.height
        msg.distortion_model = 'plumb_bob'
        
        # Distortion coefficients [k1, k2, t1, t2, k3]
        msg.d = [intrinsics.coeffs[i] for i in range(5)]
        
        # Intrinsic camera matrix (K)
        msg.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix (R) - identity için
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (P)
        msg.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        self.get_logger().info('📊 Frame İstatistikleri:')
        for stream, count in self.frame_count.items():
            self.get_logger().info(f'  {stream:15s}: {count:6d} frames')
    
    def destroy_node(self):
        """Node kapatılırken temizlik yap"""
        self.get_logger().info('🛑 RealSense Publisher Node kapatılıyor...')
        self.print_stats()
        
        if self.pipeline:
            self.pipeline.stop()
            self.get_logger().info('✅ RealSense pipeline durduruldu')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RealSensePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
