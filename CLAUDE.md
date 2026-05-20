# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Marmara University ERC 2026 rover software stack. The rover must autonomously navigate terrain using ArUco fiducial landmarks (no GNSS), avoid obstacles with YOLO, and complete ERC task modules (sampling, drilling, astrobiology). The Jetson runs ROS 2 Jazzy; the ground control station (GCS) connects via ZeroMQ over WiFi.

## Running the System

```bash
# Activate virtual environment first
source .venv/bin/activate

# Main rover control (BLDC + robot arm, non-ROS)
python Rover3.py --port /dev/ttyUSB0

# Autonomous navigation node
python autonomous_driver/autonomous_driver.py [--no-yolo] [--fake-obstacle]

# ROS 2 task nodes (requires sourced ROS 2 Jazzy)
source /opt/ros/jazzy/setup.bash
ros2 run <pkg> navigasyon      # Navigation with ArUco + GNSS
ros2 run <pkg> ornekleme       # Sampling task
ros2 run <pkg> astrobiyoloji   # Astrobiology task
ros2 run <pkg> sondaj          # Drilling task

# Ground control station GUI (PyQt5)
python Nodelar/arayuz.py

# Camera publishers
python Logitech_kamera_pub.py
python Realsense_kamera_pub.py

# ZMQ bridge (runs on Jetson, bridges ROS 2 topics to ZMQ for GCS)
python ZMQ_Bridge.py
```

## Architecture

### Communication Backbone

ZeroMQ is the primary IPC/network layer between the Jetson and GCS. Fixed port assignments — do not change without updating both sides:

| Port | Role | Direction |
|------|------|-----------|
| 5560 | Launch manager (REP) | GCS → Jetson |
| 5561 | Step motor position (PUB) | Jetson → GCS |
| 5562 | Waypoint missions (SUB) | GCS → Jetson |
| 6000–6003 | Camera streams (PUB) | Jetson → GCS |
| 6004 | System health 1 Hz (PUB) | Jetson → GCS |

### Motor Control

Two hardware layers that must **never run simultaneously** — the FSM enforces a `SETTLE_5MS` gap between transitions:

- **BLDC boards (left/right):** UART binary protocol `struct "<HhhH>"` — start=`0xABCD`, steer, speed, checksum. Speed range −1000..+1000; practical 50–150.
- **Step motors (4× steering):** UART ASCII `"MOTOR:L,R,S,L\n"`; feedback `"POS:n0,n1,n2,n3\n"`.
- **Robot arm (5× steppers + servo):** Separate mode inside `Rover3.py`.

Default serial ports: BLDC left → `/dev/ttyUSB0`, BLDC right → `/dev/ttyUSB1`, step Arduino → `/dev/ttyACM0`.

### Autonomous Driver FSM (`autonomous_driver/autonomous_driver.py`)

15-state finite state machine. Key flow:
`IDLE → NAVIGATE_INIT → SEARCH_LANDMARK → ALIGN_HEADING → APPROACH_WAYPOINT → WAYPOINT_REACHED` (repeat for each waypoint) `→ RETURN_HOME → MISSION_COMPLETE`

Emergency path: any state → `E_STOP` / `WATCHDOG_TRIGGERED` (timeout = 10 s).

Critical constants (ERC-spec derived — do not change without checking rulebook):
- `ARUCO_MARKER_LEN = 0.150` m (150 mm fiducials, ERC §7)
- `ARUCO_DICT_ID = DICT_5X5_100`
- `WAYPOINT_REACH_DIST_M = 1.5`
- `ALIGN_THRESHOLD_RAD = 0.05` (~3°)
- `MISSION_TIMEOUT_S = 1200` (20 min per ERC rules)

### Perception Pipeline

- ArUco pose estimation via `cv2.aruco` + `solvePnP` → gives 3-DOF position of each waypoint landmark.
- YOLO (Ultralytics) for obstacle detection; bbox area threshold `OBSTACLE_AREA_THRESH = 0.06` (normalized).
- RealSense D435i is the primary sensor: RGB 848×480 @ 30 Hz for inference, depth 1280×720 @ 90 Hz for obstacle distance. IMU used for tilt protection (30°/25° limits in sampling task).

### ROS 2 Node Map

```
Realsense_kamera_pub.py  →  /realsense/{rgb,depth,imu}
Logitech_kamera_pub.py   →  /logitech/image_raw
navigasyon.py            ←  /realsense/rgb + serial (GNSS, BLDC feedback)
ornekleme.py             ←  /realsense/{rgb,depth} + /logitech/image_raw
ZMQ_Bridge.py            ←  all above topics  →  ZMQ ports (to GCS)
```

## Known Bugs (documented in `autonomous_driver/docs/`)

1. **Landmark search oscillation** — direction alternates every 2 s, net rotation ≈ zero. Fix: single-direction rotation with timeout.
2. **Return-home has no localization** — drives forward for 240 s blindly. Needs EKF or odometry.
3. **Watchdog timeout mismatch** — code uses 10 s; docstring/comments say 2 s.
4. **Thread-unsafe ZMQ socket** — `step_pub` shared between FSM thread and overlay thread without a lock.
5. **Single-frame obstacle detection** — no temporal filtering; spurious detections trigger avoidance. Fix: require N consecutive frames.

## Key File Reference

| File | Purpose |
|------|---------|
| `autonomous_driver/autonomous_driver.py` | Main autonomous navigation (~2000 lines) |
| `Rover3.py` | BLDC + robot arm unified control (non-ROS) |
| `ZMQ_Bridge.py` | ROS 2 → ZMQ relay for GCS |
| `Nodelar/navigasyon.py` | ROS 2 navigation node (GNSS + ArUco) |
| `Nodelar/arayuz.py` | PyQt5 GCS with 3 cameras and telemetry plots |
| `Nodelar/ornekleme.py` | Sampling task (YOLO rock detection) |
| `autonomous_driver/docs/` | Architecture analysis, risk docs, rewrite plan |
