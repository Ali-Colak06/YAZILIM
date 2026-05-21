# navigation_guncel.py — Tespit Edilen Sorunlar

İnceleme tarihi: 2026-05-21

---

## S-1 · ArUco Dictionary Uyumsuzluğu (satır 75)

**Şiddet:** Kritik — sahada marker tespit edilemez.

```python
# Mevcut (yanlış)
self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# Olması gereken (CLAUDE.md — ERC §7)
self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
```

CLAUDE.md `ARUCO_DICT_ID = DICT_5X5_100` olarak tanımlıyor. Sahada kullanılan fiziksel markerlar 5×5_100 sözlüğüne ait; 5×5_250 sözlüğüyle eşleşme sıfır olur.

---

## S-2 · Thread-Unsafe `current_yaw` Okuma — `imu_callback` (satır 459)

**Şiddet:** Yüksek — veri yarışı, zaman zaman yanlış yaw hesaplamasına yol açar.

```python
# Sorunlu bölge
yaw_diff = math.atan2(
    math.sin(new_yaw - self.current_yaw),   # ← data_lock DIŞINDA okunuyor
    math.cos(new_yaw - self.current_yaw)
)
with self.data_lock:                         # ← içeride yazılıyor
    self.current_yaw = self.current_yaw + self.BLEND_ALPHA * yaw_diff
```

`gnss_callback` da aynı `current_yaw`'ı `data_lock` altında yazıyor. IMU callback'i ReentrantCallbackGroup içinde çalıştığından eş zamanlı erişim mümkün.

**Düzeltme:** `current_yaw` okumasını da `data_lock` bloğuna al veya önce yerel kopyasını çek.

---

## S-3 · Waypoint Ulaşma Mesafesi ERC Spec'e Aykırı (satır 550)

**Şiddet:** Yüksek — ERC puanlama kaybı.

```python
# Mevcut
if distance_error < 0.5:

# CLAUDE.md (ERC kurallarından)
# WAYPOINT_REACH_DIST_M = 1.5
if distance_error < 1.5:
```

ERC yarışma kuralları waypoint'e 1.5 m içinde ulaşmayı yeterli sayıyor. 0.5 m eşiği rover'ın hedefe çok yaklaşmaya çalışmasına, bozuk zeminde takılmasına ve zaman kaybetmesine yol açar.

---

## S-4 · `vel_linear` Odometri İçin Kullanılmıyor (satır 518-520)

**Şiddet:** Orta — başlık yorumu yanıltıcı, pozisyon kör kalıyor.

```python
vel_linear = (vel_right + vel_left) / 2.0
# Kullanım: SADECE stall tespitinde
```

Başlıkta `[BUG FIX] vel_linear kullanılmıyordu — odometri için hesaba katıldı` yazıyor ama kodda yalnızca stall sayacına giriyor; `current_x / current_y` encoder'dan güncellemiyor. GNSS kesilirse konum tahmini tamamen durur.

---

## S-5 · Stall Tespiti Sonrası Aksiyon Yok (satır 630-635)

**Şiddet:** Orta — rover takılı kalır, mission timeout'a kadar bekler.

```python
if self.stall_counter >= self.STALL_MAX:
    self.get_logger().warn(...)   # sadece log
    # ← geri manevra veya dur komutu YOK
```

`STALL_MAX` (20 iterasyon ≈ 1 s) aşıldığında aksiyon alınmıyor. Minimum beklenti: `send_to_hoverboard(0.0, 0.0)` ile durdur; ideal: kısa geri manevra.

---

## S-6 · `align_depth_callback` Boş (satır 333-335)

**Şiddet:** Düşük — gereksiz kaynak tüketimi.

```python
def align_depth_callback(self, msg):
    pass
```

Depth topic'ine subscribe ediliyor ama callback hiçbir şey yapmıyor. Her kare (30 Hz) için topic kopyalama ve callback tetikleme maliyeti boşa gidiyor. Topic'e şimdilik subscribe olunmamalı; engel algılama eklendiğinde açılmalı.

---

## Özet Tablosu

| # | Sorun | Şiddet | Satır |
|---|-------|--------|-------|
| S-1 | ArUco sözlük yanlış (250 → 100) | Kritik | 75 |
| S-2 | `current_yaw` lock dışı okunuyor | Yüksek | 459 |
| S-3 | Waypoint eşiği 0.5 m (olması gereken 1.5 m) | Yüksek | 550 |
| S-4 | `vel_linear` odometride kullanılmıyor | Orta | 518-520 |
| S-5 | Stall sonrası aksiyon yok | Orta | 630-635 |
| S-6 | Depth callback boş, subscribe gereksiz | Düşük | 333-335 |
