# `autonomous_driver.py` — Risk, Bug ve Gap Analizi

> Bu rapor, `autonomous_driver_analysis.md`'in **11-13. bölümlerini** içerir: failure modes, bug listesi, code smells ve "korunacak / refactor edilecek / baştan yazılacak" parça ayrımı.

---

## 11. Failure Mode ve Safety Analizi

> Aşağıdaki tabloda her arıza durumu için **mevcut davranış**, **yeterlilik** ve **iyileştirme** verilmiştir. "Mevcut davranış" doğrudan koddan çıkartılmıştır.

| # | Senaryo | Mevcut Davranış (kod referansı) | Yeterli mi? | İyileştirme |
|---|---------|--------------------------------|-------------|------------|
| 1 | Kamera hiç gelmiyor | Watchdog Layer B (1342-1348): `_camera_start_ts`'ten WATCHDOG_TIMEOUT_S sonra `trigger_watchdog()` | Kısmen — `WATCHDOG_TIMEOUT_S=10s`, ERC için yavaş | Timeout 1.5-3s, sensör başına ayrı watchdog |
| 2 | Kamera stale (geç) | Layer B (1349-1354): timestamp farkı kontrol | OK | OK |
| 3 | Depth yok ama RGB var | AvoidancePlanner default boyutu 848×480 kullanır (951-952), depth-based switch atlanır → bbox/seg fallback | Çalışır ama sessiz | Telemetri bayrağı: "depth absent" health'a düşmeli |
| 4 | YOLO modeli yok | `--no-yolo` veya model None: `_EMPTY_DET/_EMPTY_SEG` → planner DRIVE | Tehlikeli — engelsiz sürer | YOLO yoksa "manual mode only" zorunlu, FSM IDLE'de kalsın |
| 5 | YOLO inference yavaş | YoloWorker thread bloklanır, FSM stale veri okur | Tolere edilebilir | Inference timeout, frame-skip stratejisi |
| 6 | ArUco görünmüyor | `_aruco_lost_ticks` 25 tick (~0.5s) sonra SEARCH_LANDMARK | OK fakat SEARCH bug'lı (yön salınımı) | SEARCH algoritması düzelt |
| 7 | Yanlış ID'li ArUco | `ARUCO_VALID_IDS = range(0,65)` → ID 0-64 hepsi kabul; **yorum 51-64 diyor** | **Bug** | Set'i `range(51, 65)` olarak daralt |
| 8 | Marker geçici kaybı (occlusion) | 25 tick tolerans | OK | Tracker (KCF/CSRT) eklenebilir |
| 9 | Kamera kalibrasyonu yanlış | Hardcoded matrix (D435i 848×480). Yanlış çözünürlükte pose tahmini sapar | Kötü | YAML calib + runtime check |
| 10 | Serial port açılmıyor | `_open_ports` `_log.error`, `Serial=None`. send_bldc/step sessiz no-op | Tehlikeli (komutlar dropping) | Açılamayan port → fatal `_do_terminal(E_STOP)` veya manual mode |
| 11 | BLDC feedback bozuk | `_parse_feedback` checksum başarısız → return; eski feedback kalır. FSM zaten kullanmıyor. | OK ama veri kullanılmıyor | Feedback mandatory, FSM'de tüket |
| 12 | Step encoder yok | `capture_home` 3s timeout sonra `home_pos=[0,0,0,0]` | Tehlikeli — E-stop home dönüşü yanlış | Encoder zorunlu; yoksa E-stop sadece "hold" |
| 13 | ZMQ mesajı bozuk | `WaypointReceiver` `_log.warning('Geçersiz mesaj')`; `LaunchManager` JSON parse hatası → success:False | OK | Schema validation (pydantic/jsonschema) |
| 14 | Abort gelir | FSM tick başında `pop_abort()` → `_do_terminal(E_STOP)` | OK | OK |
| 15 | Watchdog yanlış tetikleme | `last_update_ts > 0` guard var (Layer A) → ilk paket gelene dek tetiklemiyor; kamera için `_camera_start_ts` kullanılıyor | Kısmen OK | Kamera için 'first frame received' bayrağı eklenmeli |
| 16 | Watchdog geç tetiklenir | 5 Hz kontrol + 10s timeout → en kötü 10.2s gecikme | ERC için yavaş | 10 Hz + 2s timeout |
| 17 | FSM state'te takılır | `_s_search_landmark` timeout=120s → MISSION_COMPLETE; APPROACH/ALIGN için timeout yok | **Eksik** | Her state için max-duration |
| 18 | Engel kararı yanlış (false positive) | TURN'e gidip 500ms döner; sonra SCAN; donanım hasarı yok | OK ama waypoint kaybı | Persistent obstacle tracking; hareket eden engel ayrımı |
| 19 | Engel false negative | Çarpışma | Felaket | Çoklu sensör (LiDAR/ultrasonic) füzyonu |
| 20 | Step + BLDC aynı anda komut | TURN→SETTLE_5MS→DRIVE sıralaması garanti; ama ALIGN_HEADING ve APPROACH_WAYPOINT BLDC çalıştırıyor; bu sırada step motor 'S' kalmalı | Kod 'S' bırakıyor (TURN sonunda) | İnvariant'ı kod düzeyinde tip ile garanti et (`DriveCmd` builder) |
| 21 | E-stop sırasında home dönüşü başarısız | Encoder yanıt yoksa STOP fallback; başarılıysa tolerance kontrolü | Kısmen OK | Mekanik fren komutu eklemek; brake-on-power-loss |
| 22 | Mission timeout | `_mission_start_ts`'ten 20dk → MISSION_COMPLETE | OK (ERC kuralı) | Per-waypoint timeout da gerekli |
| 23 | RETURN_HOME yanlış | Ev konumuna dönmüyor; sadece ileri sürer 240s | **Yanlış davranış** | Gerçek lokalizasyon (V-SLAM/odom/IMU integral) |
| 24 | YOLO çıktısı stale | `_det = self._det` lock altında en son sonuç; FSM bunu kullanır → en kötü 100ms eski | OK | Timestamp ekle, stale ise drop |
| 25 | Marker pose flip (PnP ambiguity) | Yok — solvePnP tek çözüm dönüyor; flip riski tek-marker'da var | Pratik az; kötü açıdan yaklaşımda yanılır | İki-frame consistency check, IPPE_SQUARE iki çözüm dönerse uygun olanı seç |
| 26 | Sürücü reverse durum | Yok — geri sürüş hiçbir yerde yok | İyileştirme | Bazı engellerde reverse + reroute |
| 27 | Aşırı yamuk arazide | PID + tek-frame planner stabilite kaybeder | İyileştirme | IMU ile roll/pitch kontrolü, tilt alarmı |
| 28 | Çoklu marker görünüyor (yanlış hedef) | `dets[mid]` dict'inden hedef ID ile arama → doğru hedefi alır | OK | OK |
| 29 | Marker 2m'den çok uzaktan görüldü | yaklaş+PID; ama bbox küçükse heading_err gürültülü | İyileştirme | Mesafe-bazlı confidence weight |
| 30 | ZMQ port çakışması | Bind hatası `_log.warning` → publishing yok ama akış devam eder | Kısmen | Hata fatal yapılmalı veya alternatif port |
| 31 | rclpy.init() hata | exception → main() çöker | OK ama kullanıcıya net mesaj eksik | try/except + structured shutdown |
| 32 | OS sinyali (SIGINT) | `_on_signal` E-stop tetikler; main while loop fsm.done bekler → execute_estop tamamlanmadan çıkış mümkün | Kısmen | Sinyal sonrası grace period |
| 33 | Disk dolu (logging) | Logger STDOUT (FileHandler yok) | OK | Rotating file logger eklenmeli |
| 34 | RAM yetersiz / OOM | Numpy frame copy + YOLO → OOM mümkün; özel koruma yok | İyileştirme | Resource monitor thread |
| 35 | Aynı ZMQ context multiprocess paylaşımı | `zmq.Context.instance()` thread-shared OK; multiprocess olmadığı için sorun yok | OK | OK |
| 36 | Power loss | Bağımsız donanım fail-safe (BLDC kart firmware'i 0 hız, step motor brake) | Kod kapsamı dışı | Donanım fail-safe doğrulanmalı |

### 11.1. ERC §7.3.2.1'e Özgü Failure Senaryoları

| ERC Risk | Mevcut Kod | Eksiklik |
|----------|-----------|----------|
| 4 waypoint herhangi sırayla | Sadece sırayla deniyor; bulamadığı ilk hedefte MISSION_COMPLETE | Hedefler arası "skip → revisit" stratejisi |
| 20 dk pencere | MISSION_TIMEOUT_S=1200s ✓ | OK |
| GNSS yok bonus | Tamamen görsel ✓ | Lokalizasyon tek başına ArUco'ya bağlı; marker yoksa kod kayboluyor |
| Site keşfi (drone önceden 10 dk) | Kapsam dışı | İleri faz: drone'dan koordinat alma |

---

## 12. Bug, Code Smell ve Tasarım Zayıflığı Listesi

> Her madde: **konum**, **sorun**, **risk**, **düzeltme**, **yeni tasarımda nasıl olmalı**.

### 12.1. Kritik Bug'lar (BLOCKER)

#### BUG-1 — `ARUCO_VALID_IDS` yanlış range
- **Konum**: satır 115. `ARUCO_VALID_IDS = frozenset(range(0, 65))`.
- **Sorun**: Yorumda "ID 51..64" yazılıyor ama frozenset 0-64'ü kabul ediyor. ERC harici/yanlış ID'ler de geçerli sayılır.
- **Risk**: Yarışmada başkalarına ait dummy markerlar veya ortamda bulunan diğer 5x5 işaretçiler hedef sayılabilir.
- **Düzeltme**: `frozenset(range(51, 65))` (51 dahil, 65 hariç → 51-64).
- **Yeni tasarım**: ID listesi config'ten gelmeli (örn. `config/erc2026.yaml`).

#### BUG-2 — `_s_search_landmark` yön salınımı
- **Konum**: satır 1607-1617.
- **Sorun**: `_scan_turn_dir = TURN_LEFT if RIGHT else RIGHT` her 2 saniyede yön değişiyor. Net açısal yer değiştirme = 0; rover yerinde sallanır.
- **Risk**: ArUco'yu gerçekten "tarayamaz". 120s timeout sonra MISSION_COMPLETE.
- **Düzeltme**: Tek yönde sürekli dön. Tam 360° tamamlandıktan sonra konum değiştir → tekrar 360°.
- **Yeni tasarım**: Tarama state machine'i: `SCAN_ROTATE → SCAN_TRANSLATE → SCAN_ROTATE`. IMU yaw integration ile tam 360 garantile.

#### BUG-3 — `_s_return_home` lokalizasyon yok
- **Konum**: satır 1716-1741.
- **Sorun**: "Eve dönüş" sadece "engel yoksa ileri" + 240s timeout. Hangi yöne gittiği belirsiz; rotasyon yok.
- **Risk**: Rover daha önce sürüldüğü yönden tamamen farklı yöne ileri sürer; kaybolur.
- **Düzeltme**: ya RETURN_HOME state'ini sil ve sadece MISSION_COMPLETE'e geçir, ya da gerçek odom/SLAM/path-replay implementasyonu yap.
- **Yeni tasarım**: `localization/` modülü; visual odometry + IMU fusion. Path memory.

#### BUG-4 — Search timeout sonrası mission tamamen biter
- **Konum**: satır 1601-1604.
- **Sorun**: 4 hedeften ilki bulunamazsa kalan 3'ü hiç denenmeden MISSION_COMPLETE.
- **Risk**: ERC puanı kaybolur (her hedef 25 puan).
- **Düzeltme**: Hedefi listenin sonuna at, sıradakiyle dene. Tüm hedefler bulunamadıysa NCO tamamla.

#### BUG-5 — `WATCHDOG_TIMEOUT_S` ve docstring çelişkisi
- **Konum**: satır 135 vs 1306-1307.
- **Sorun**: Sabit 10s, docstring "2s". 10s hem ERC reaktif emniyeti için yavaş hem de açıklama yanıltıcı.
- **Düzeltme**: 1.5-3s aralığına çek; docstring güncel kal.

#### BUG-6 — IDLE state'inden otomatik SCAN→DRIVE geçişi
- **Konum**: satır 1513-1515.
- **Sorun**: Mission gelmeden YOLO sonucu hazır olunca SCAN'e geçiyor → SCAN→DRIVE→sürmeye başlıyor.
- **Risk**: Rover görev olmadan kendiliğinden hareket eder → tehlikeli, beklenmedik davranış.
- **Düzeltme**: IDLE'den çıkış sadece mission alındığında olmalı. SCAN/DRIVE bağlamı navigasyon altına taşınmalı.

#### BUG-7 — `step_pub` ZMQ socket thread-safe değil
- **Konum**: main() satır 1934-1941, FSM `_step_zmq.send_string` (1479), `OverlayPublisher.set_step_pub` (1019).
- **Sorun**: ZMQ docs net: socket'ler thread-safe değildir. Şu anda overlay socket'i kullanmıyor ama referans tutuyor. İleride biri kullanırsa segfault.
- **Düzeltme**: Her thread kendi socket'ini açsın; veya tek bir publisher thread'i tüm send'leri tek-noktadan yapsın.

#### BUG-8 — ALIGN_HEADING'de engel kontrolü yok
- **Konum**: satır 1619-1648.
- **Sorun**: Yerinde dönerken yan-engel YOLO'da görünmez (front-band'in dışında olur), ama yine de fiziksel temas riski var. Üstelik durum bir kullanıcı yan tarafta ise engel sayılmaz.
- **Risk**: ALIGN sırasında dönüş çapı içinde insan/cihaz çarpılabilir.
- **Düzeltme**: ALIGN'de de planner çalıştır, engelse SCAN'e dön.

### 12.2. Önemli Bug'lar (HIGH)

#### BUG-9 — UART read state machine `prev` reset yok
- **Konum**: satır 397-424. Frame tamamlandıktan sonra `r['prev']` set edilmez (doğrusu None'a çekilmeli).
- **Risk**: Ardışık frame'ler arasında yanlış senkron.
- **Düzeltme**: Frame tamamlandıktan sonra `r['prev'] = None`.

#### BUG-10 — Per-tick step motor komutu spamı (TURN state)
- **Konum**: satır 1536-1546.
- **Sorun**: 50 Hz × 500 ms = 25 kez "MOTOR:L,L,L,L\n" gönderilir; Arduino UART buffer şişir, gecikme.
- **Düzeltme**: Komutu sadece state girişinde bir kez gönder; süre sonu durma komutuyla bitir.

#### BUG-11 — `_DRIVE_*` constant'lar ölü kod
- **Konum**: satır 216-219.
- **Sorun**: `DriveCmd` modeli ve preset'ler tanımlanmış ama FSM doğrudan `bus.send_bldc/send_step_motors` çağırıyor. `MotorBus.send_drive` da kullanılmıyor.
- **Düzeltme**: Tüm motor komutları `DriveCmd` üzerinden geçmeli (atomic, audit-friendly).

#### BUG-12 — Telemetri tüketici yok
- **Konum**: `TelemetrySubscriber.state` hiçbir yerde okunmuyor.
- **Sorun**: IMU/batarya/akım verileri var ama FSM kullanmıyor.
- **Risk**: Düşük batarya, yüksek akım, yan-yatma alarmı yok.
- **Düzeltme**: Watchdog'a batarya/akım eşiği ekle; FSM'de tilt-protection.

#### BUG-13 — `OverlayPublisher` bind hatası sessiz
- **Konum**: satır 1043-1044. `try/except zmq.ZMQError as exc: self._log.warning(...)`.
- **Risk**: Operator port açılmadığını fark etmez, ground station siyah ekran.
- **Düzeltme**: bind başarısızsa health'a "publish_failed" durumu yaz; cli'da fatal.

#### BUG-14 — `DetectionResult/SegmentationResult` mutable
- **Konum**: satır 627-639. Frozen değil.
- **Risk**: YOLO worker yazdıktan sonra başka thread mask listesi'ni in-place değiştirirse race.
- **Düzeltme**: `frozen=True` veya immutable read-only view.

#### BUG-15 — PID dt ilk iterasyonda 0.02 hard-code
- **Konum**: satır 906-913. `dt = 0.02` ilk iterasyon için.
- **Risk**: 50 Hz'de doğru ama freq değişirse hatalı türev.
- **Düzeltme**: İlk update derivative=0 kabul et veya `_prev_ts = now` yap → next loop gerçek dt.

#### BUG-16 — Per-state timeout yok (search dışında)
- **Konum**: ALIGN_HEADING, APPROACH, RETURN_HOME, IDLE.
- **Risk**: Rover tek bir state'te sonsuz takılabilir (sadece mission timeout 20dk kurtarır).
- **Düzeltme**: Her state için entry timestamp + max_duration kontrolü.

### 12.3. Code Smells (MEDIUM)

#### SMELL-1 — Tek dosyada 2023 satır, çoklu sorumluluk
- Architecture, hardware abstraction, ROS2, perception, control, comm, FSM, main hepsi tek dosyada. Test edilemez.

#### SMELL-2 — Hard-coded magic number'lar
- `OBSTACLE_AREA_THRESH=0.06`, `FRONT_CENTER_BAND=(0.35, 0.65)`, kamera matrisi, tüm portlar/topic'ler.
- Refactor'da config dosyası (YAML/TOML) zorunlu.

#### SMELL-3 — Sessizce `pass` edilen exception'lar
- `OverlayPublisher._send_jpg` `except: pass` (1122-1123).
- `OverlayPublisher._send_health` `except: pass` (1154-1155).
- `WaypointReceiver._lock altındaki recv_string` exception sessiz log+sleep.
- Hata yutmak debug edilebilirliği yok eder.

#### SMELL-4 — Test edilebilirlik yok
- ROS2/UART/ZMQ doğrudan main'de bağlanıyor; mock injection yok.
- Birim test sıfır.

#### SMELL-5 — `getattr(self, f'_s_{state.name.lower()}')` reflection ile dispatch
- Type-checker faydası kayboluyor; rename yapılırsa runtime'da bulunmaz.
- Açık dispatch table veya state pattern kullanılmalı.

#### SMELL-6 — Aynı state geçiş kodu birden fazla yerde tekrarlı
- Engel→TURN_PREP geçişi SCAN, CHECK_OBSTACLE, APPROACH_WAYPOINT, RETURN_HOME'da kopyalanmış.
- "request_turn(action, post_settle)" helper'ı eksik.

#### SMELL-7 — `Optional` kullanımı tutarsız
- Bazı yerlerde `Optional[...] = None`, bazı yerde `obj or None` koşulları.

#### SMELL-8 — Logger kullanım tutarsızlığı
- Class-level `_log = logging.getLogger('otonom.xxx')`, modül-level `log = logging.getLogger('otonom')`. Karmaşık.

#### SMELL-9 — `_post_settle_state` global FSM state
- Birden fazla TURN_PREP girişinin "nereye dönecek" bilgisini taşıyor; bağlam encoding'i implicit.
- Refactor: TURN_PREP transition'ı argümanla parametrik state olmalı.

#### SMELL-10 — Comments/code drift
- "C-1 düzeltmesi", "C-2 düzeltmesi" yorumları geçmiş incident'lara referans; commit history'de kalsın, kod yorumlarından silinsin.

### 12.4. Tasarım Zayıflıkları (DESIGN)

#### DESIGN-1 — Sensor fusion / localization yok
- Sadece ArUco + RealSense. IMU yok; visual odometry yok; SLAM yok.
- Marker görünmüyorsa rover hiçbir şey yapamıyor.

#### DESIGN-2 — Basit obstacle avoidance
- Tek-frame karar; tracking yok; hareket eden engel yok; planlama yok.

#### DESIGN-3 — RETURN_HOME basit ileri sürüş
- Bug-3'le aynı.

#### DESIGN-4 — CANMotorBus yok
- Tamamen stub. Multi-bus mimari iddia edilse bile kullanılamıyor.

#### DESIGN-5 — Thread lifecycle problemleri
- Stop event'leri var ama join çağrılmıyor; daemon thread'ler interpreter exit'inde silinir, kapanış sırası belirsiz.

#### DESIGN-6 — Safety-critical eksikleri
- Tilt protection yok.
- Battery low → graceful return yok.
- Stuck detection yok (kerevet/çukurda dönmeye çalışırken).
- Heartbeat to ground station yok (tek yönlü health pub).

#### DESIGN-7 — Configuration management
- YAML / TOML / env config yok. Argparse ile sınırlı (sadece port'lar).

#### DESIGN-8 — Logging/diagnostics yetersiz
- File logger yok; structured logging (JSON) yok; metrics export yok (Prometheus, vs).

#### DESIGN-9 — Atomic command guarantee weak
- Step + BLDC mutex sadece FSM disiplini ile sağlanıyor; type-system veya mutex ile değil.

#### DESIGN-10 — Time source tutarsız
- `time.time()` ve `time.monotonic()` karışık. Wall-clock NTP-shift bug'ı potansiyeli.

---

## 13. Korunacak / Refactor / Baştan Yaz Tabloları

### 13.1. Korunabilir Parçalar

| Parça | Konum | Neden | Öneri |
|-------|-------|-------|-------|
| `MotorBus` (ABC) arayüzü | 226-270 | İyi düşünülmüş soyutlama; UART/CAN switch için temel | `home_pos`'u abstract property yap; `Drive` enum eklenebilir |
| BLDC paket protokolü (struct `<HhhH>`) | 105 | Rover.py ile aynı; firmware uyumlu | Sabit |
| `ArucoWorker._OBJ_PTS` ve solvePnP IPPE_SQUARE seçimi | 786-791, 855-858 | Doğru köşe sıralaması, hızlı planar PnP | Olduğu gibi taşı |
| `ArucoDetection` dataclass | 198-204 | Frozen, anlamlı alanlar | rvec + reprojection_err ekle |
| `WaypointList` dataclass | 207-212 | Frozen, mission abstraction temiz | per-waypoint config genişlet |
| `CameraHub` ROS2 abone deseni | 584-620 | Lambda binding doğru | Topic'i config'e taşı |
| FSM state diagram konsepti | 1363-1382 | İskelet sağlam | Hiyerarşik FSM/BT'ye refactor |
| `execute_estop` per-motor home dönüşü | 1241-1296 | C-2 düzeltmesinden sonra mantık doğru | Modülerleştir, FSM içine al |
| Watchdog 2-katmanlı yapı | 1303-1356 | Konsept iyi | Kapsam genişlet (sensör, akım, batarya) |
| Annotated overlay JPEG yayını | 1115-1123 | GS uyumlu protokol | Resmi telemetri çerçevesinde tut |
| ZMQ port haritası (5560/5561/5562/6000-6004) | 154-166 | GS ile sözleşme | Sabit; config dışına çıkar |
| `--fake-obstacle` test mode | 760-769 | Saha-dışı test için pragmatik | Genişlet (fake-aruco, fake-telemetry) |

### 13.2. Refactor Edilmesi Gerekenler

| Parça | Neden | Önerilen Aksiyon |
|-------|-------|-------------------|
| `UARTMotorBus` | Tek dosyada 3 thread + parser + writer; sessiz hata yutma | `hardware/uart_bus.py` ayrı; reader/writer ayrı thread; structured error reporting |
| `YoloWorker` | GIL/inference latency, GPU memory yönetimi yok | Process-pool worker; preprocessing pipeline; warmup; class-name → semantic mapping |
| `AvoidancePlanner` | If/else ladder; tek-frame; tracking yok | Costmap (occupancy grid) tabanlı + tracking + lokal planner (DWA/TEB lite) |
| `OverlayPublisher` | Render + health birarada; bind hata sessiz | `diagnostics/health_publisher.py` ve `diagnostics/overlay.py` ayrı; bind hata fatal/escalate |
| `WaypointReceiver` | ID listesi only; abort flag | Mission DSL (waypoint = id + tip + approach + timeout); abort event |
| `Watchdog` | 5 Hz, sadece 2 katman | 10 Hz, multi-source, FSM tick freshness, batarya, akım, IMU sağlığı |
| `AutonomousFSM` | 2k satırlık monolith içinde 600+ satır; reflection dispatch | `fsm/states/` per-state file; explicit transition table; type-safe context |
| `PIDController` | Anti-windup, derivative filter yok | Filtered PID + integral clamp; derivative on measurement |
| `TelemetrySubscriber` | State tüketilmiyor | FSM, Watchdog, OverlayPub için unified telemetry bus |
| `main()` | 160 satır setup; sıralama implicit | Composition root + dependency injection (örn. `runtime/launcher.py`) |
| Logger setup | Console only | Structured logging (JSON) + RotatingFileHandler + log level config |

### 13.3. Baştan Yazılması Gerekenler

| Parça | Neden | Yeni Yer |
|-------|-------|----------|
| `CANMotorBus` | Sadece stub | `hardware/can_bus.py` (python-can asyncio) |
| `_s_search_landmark` algoritması | Bug-2: yön salınımı, etkisiz tarama | `navigation/landmark_search.py`: tek-yön döngü + IMU yaw integration |
| `_s_return_home` | Lokalizasyon yok; bug-3 | `localization/` + `navigation/return_home.py`: path-memory or visual odometry |
| Configuration system | Yok; her şey hard-coded | `config/*.yaml` + pydantic schema + dotenv |
| Test harness | Yok | `tests/unit`, `tests/integration`, sim/mock motor bus |
| Localization | Yok | `localization/`: IMU + visual odom + ArUco landmark fusion (EKF) |
| Mission planning | ID listesi sırayla | `planning/mission_planner.py`: traveling-salesman heuristic, retry, skip |
| Recovery behaviors | Yok | `safety/recovery.py`: stuck detection, reverse-and-retry, escalation |
| Diagnostics export | health JSON 1 Hz | `diagnostics/`: Prometheus exporter + structured logs + replay buffer |
| Mock hardware | Yok (sadece `--no-yolo`, `--fake-obstacle`) | `tests/fakes/`: FakeMotorBus, FakeCamera, FakeAruco, FakeWaypointPublisher |

---

> Bölüm 14 ve sonrası `docs/autonomous_driver_rewrite_plan.md` dosyasındadır.
