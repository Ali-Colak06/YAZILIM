#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image       
from std_msgs.msg import String        
from cv_bridge import CvBridge        
import cv2
import os

class QROkuyucuNode(Node):
    def __init__(self):
        super().__init__('qr_okuyucu_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/realsense/rgb/image_raw',  
            self.kamera_callback,       
            10                           # Kuyruk boyutu
        )
        
        self.qr_publisher = self.create_publisher(String, '/astrobiology/qr_result', 10)
        
        self.frame_sayaci = 0
        self.kac_framede_bir_oku = 3  
        
        bulundugum_klasor = os.path.dirname(os.path.abspath(__file__))
        models_klasoru = os.path.join(bulundugum_klasor, "qrModels")
        
        detect_prototxt = os.path.join(models_klasoru, "detect.prototxt")
        detect_caffe = os.path.join(models_klasoru, "detect.caffemodel")
        sr_prototxt = os.path.join(models_klasoru, "sr.prototxt")
        sr_caffe = os.path.join(models_klasoru, "sr.caffemodel")
        
        try:
            self.wechat_detector = cv2.wechat_qrcode_WeChatQRCode(
                detect_prototxt, detect_caffe, sr_prototxt, sr_caffe
            )
            self.get_logger().info("WeChat QR Modelleri yüklendi. Realsense bekleniyor...")
        except Exception as e:
            self.get_logger().error(f"Model Yükleme Hatası: {e}")
            self.get_logger().error("Lütfen 'models' klasörünün ve içindeki 4 dosyanın varlığından emin olun.")

    def kamera_callback(self, msg):
        """Realsense'ten her yeni görüntü geldiğinde bu fonksiyon OTOMATİK çalışır"""
        
        # İşlemciyi yormamak için kare atlatma 
        self.frame_sayaci += 1
        if self.frame_sayaci % self.kac_framede_bir_oku != 0:
            return

        try:
            # 1. ROS Mesajını OpenCV Görüntüsüne (Renkli - BGR8) Çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Görüntüyü WeChat Yapay Zekasına Ver
            # res: Okunan metinleri liste olarak verir (Örn: ["Merhaba", "Mars"])
            # points: QR kodların köşelerinin piksel koordinatlarını verir
            res, points = self.wechat_detector.detectAndDecode(cv_image)
            
            # 3. Eğer ekranda QR varsa
            if len(res) > 0:
                for qr_metni in res:
                    if qr_metni: # Boş değilse
                        self.get_logger().info(f"QR TESPİT EDİLDİ: {qr_metni}")
                        
                        # Veriyi ROS ağına bağırıyoruz (Arayüz veya Astrobiyoloji node'u duysun diye)
                        mesaj = String()
                        mesaj.data = qr_metni
                        self.qr_publisher.publish(mesaj)
                        
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    dugum = QROkuyucuNode()
    
    try:
        rclpy.spin(dugum) # Node'u sonsuz döngüde çalışır halde tutar
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
