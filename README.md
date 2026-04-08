# 🤖 eYRC Holo Battalion — Multi-Robot Autonomous Pick & Place System

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10+-green?logo=python)
![ESP32](https://img.shields.io/badge/ESP32-Firmware-orange?logo=espressif)
![MQTT](https://img.shields.io/badge/MQTT-Paho-purple)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red?logo=opencv)

**Team ID:** HB_2442 &nbsp;|&nbsp; **Theme:** eYRC Holo Battalion

*A fully autonomous multi-robot coordination system for coordinated pick-and-place operations with pyramid stacking, using holonomic drive robots, ArUco-based localization, and MQTT-bridged ESP32 hardware.*

</div>

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [Team Members](#-team-members)
3. [System Architecture](#-system-architecture)
4. [Repository Structure](#-repository-structure)
5. [Hardware Components](#-hardware-components)
6. [Software Stack](#-software-stack)
7. [Arena Layout & Zones](#-arena-layout--zones)
8. [Communication Pipeline](#-communication-pipeline)
9. [Node Descriptions](#-node-descriptions)
10. [Algorithms & Logic](#-algorithms--logic)
11. [ROS 2 Topics & Services](#-ros-2-topics--services)
12. [ESP32 Firmware](#-esp32-firmware)
13. [Setup & Installation](#-setup--installation)
14. [Running the System](#-running-the-system)
15. [Success Criteria](#-success-criteria)
16. [Known Constraints](#-known-constraints)

---

## 🌟 Project Overview

The **eYRC Holo Battalion** project demonstrates coordinated autonomous operation of **three holonomic drive robots** performing pick-and-place tasks in a shared arena. The robots collaborate to:

- 🔍 **Detect** crates using overhead ArUco marker localization
- 🧠 **Plan** collision-free paths using the Theta\* any-angle planner
- 🦾 **Pick** crates using a servo arm + electromagnetic gripper
- 📦 **Transport** crates to color-coded drop zones
- 🏗️ **Stack** crates in a pyramid formation
- 🏠 **Return** to their designated docking positions

All inter-robot coordination is managed by a single **leader robot** (ID 0, `hb_crystal`) using a Hungarian Algorithm-based task allocator, with zone locking to prevent simultaneous access conflicts.

---

## 👥 Team Members

| Name | Role |
|------|------|
| **Kumar Sushant Raj** | Perception & State Machine  |
| **Atharva Rana** | Path Planning & Task Allocation  |
| **Aditya Kumar** | Harware & Obstacle Avoidance |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        LAPTOP / ROS2 HOST                           │
│                                                                     │
│  ┌──────────────┐    /bot_pose     ┌─────────────────────────────┐  │
│  │  Perception  │ ──────────────►  │                             │  │
│  │    Node      │    /crate_pose   │  MultiHolonomicController   │  │
│  │  (ArUco +    │ ──────────────►  │  (Per Robot Instance)       │  │
│  │  Homography) │                  │                             │  │
│  └──────────────┘                  │  • Task Allocator (Leader)  │  │
│        ▲                           │  • Theta* Path Planner      │  │
│        │ /camera/image_raw         │  • PID Controllers          │  │
│  ┌─────┴──────┐                    │  • State Machine            │  │
│  │  Overhead  │                    │  • Zone Lock Manager        │  │
│  │  Camera    │                    └──────────┬──────────────────┘  │
│  └────────────┘                              │ /bot_cmd             │
│                                              ▼                      │
│                                  ┌───────────────────────┐          │
│                                  │    MQTT Bridge Node   │          │
│                                  │  (ROS2 ↔ MQTT)        │          │
│                                  └───────────┬───────────┘          │
│                                              │ MQTT (WiFi)          │
└──────────────────────────────────────────────┼──────────────────────┘
                                               │
                    ┌──────────────────────────┼────────────────────┐
                    │         ESP32 (per robot) │                   │
                    │  ┌────────────────────────▼──────────────┐    │
                    │  │  MQTT Subscriber → Motor Driver +      │   │
                    │  │  Servo Controller + Solenoid Control   │   │
                    │  └───────────────────────────────────────┘    │
                    │          ▲ IR Sensor Data (esp/sensor/1)      │
                    └───────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
holo_battalion/
│
├── 📄 MultiHolonomicController.py   # Main controller node (all robots)
├── 📄 perception.py                 # ArUco localization node
├── 📄 mqtt_bridge_node.py           # ROS2 ↔ MQTT bridge
├── 📄 esp32_firmware/
│   └── firmware.ino                 # ESP32 Arduino firmware
│
├── 📄 README.md                     # This file
└── 📄 requirements.txt              # Python dependencies
```

---

## 🔧 Hardware Components

| Component | Description |
|-----------|-------------|
| 🤖 **3× Holonomic Drive Robots** | 3-wheeled omnidirectional drive (`hb_glacio`, `hb_crystal`, `hb_frostbite`) |
| 🦾 **Servo Arm** | 2-DOF arm with base and elbow servo joints |
| 🧲 **Electromagnetic Gripper** | Solenoid-based magnet for crate attachment |
| 🔦 **IR Proximity Sensor** | Detects crate contact for reliable pickup triggering |
| 📡 **ESP32 Microcontroller** | WiFi-enabled MCU onboard each robot |
| 📷 **Overhead Camera** | Top-down view for full-arena ArUco marker localization |

---

## 🖥️ Software Stack

| Layer | Technology |
|-------|------------|
| **Framework** | ROS 2 (Humble) |
| **Language** | Python 3.10+ |
| **Vision** | OpenCV 4.x + ArUco Markers (`DICT_4X4_250`) |
| **Path Planning** | Theta\* (any-angle A\*) |
| **Task Allocation** | Hungarian Algorithm (`scipy.optimize.linear_sum_assignment`) |
| **Control** | PID Controllers (X, Y, θ) |
| **MQTT Broker** | Mosquitto / Paho-MQTT |
| **Firmware** | Arduino (ESP32) + PubSubClient |
| **Interfaces** | Custom `hb_interfaces` ROS 2 package (`BotCmd`, `BotCmdArray`, `Poses2D`) |

---

## 🗺️ Arena Layout & Zones

The arena is a **2438.4 mm × 2438.4 mm** (8 ft × 8 ft) square, with the origin at the **top-left** corner. The Y-axis points **downward**.

### 📍 Robot Docking Positions

| Robot | X (mm) | Y (mm) | Yaw |
|-------|--------|--------|-----|
| `hb_glacio` | 864.0 | 204.0 | 0° |
| `hb_crystal` | 1218.0 | 205.0 | 0° |
| `hb_frostbite` | 1568.0 | 202.0 | 0° |

### 📦 Drop Zones

| Zone | Color | X Min | X Max | Y Min | Y Max |
|------|-------|-------|-------|-------|-------|
| **D1** | 🔴 Red | 1020 | 1410 | 1075 | 1355 |
| **D2** | 🟢 Green | 675 | 965 | 1920 | 2115 |
| **D3** | 🔵 Blue | 1470 | 1762 | 1920 | 2115 |

### 🎨 Crate Color Assignment

Crate color is determined by the **ArUco marker ID modulo 3**:

```
Crate ID % 3 == 0  →  🔴 RED   →  Drop Zone D1
Crate ID % 3 == 1  →  🟢 GREEN  →  Drop Zone D2
Crate ID % 3 == 2  →  🔵 BLUE   →  Drop Zone D3
```

**Active crate IDs:** `{12, 13, 14, 16, 21, 30}`

---

## 📡 Communication Pipeline

The system follows a **layered communication model**:

```
Overhead Camera
      │
      ▼ (ROS Image Topic)
Perception Node  ─────────────►  /bot_pose  (Poses2D)
                 ─────────────►  /crate_pose (Poses2D)
                                      │
                                      ▼
                          MultiHolonomicController
                          (Task Allocator + PID + Planner)
                                      │
                                      ▼ /bot_cmd (BotCmdArray)
                              MQTT Bridge Node
                                      │
                                      ▼ MQTT (WiFi, TCP 1883)
                                   ESP32
                                      │
                              ┌───────┴────────┐
                           Motors          Solenoid + Servos
                                      │
                                      ▼ MQTT (esp/sensor/1)
                              IR Sensor Reading
                                      │
                                      ▼ /ir_sensor_status (Bool)
                          MultiHolonomicController
```

---

## 🧩 Node Descriptions

### 1. 🧠 `MultiHolonomicController` (`MultiHolonomicController.py`)

The core node — one instance runs **per robot**, parameterized by `robot_name` and `robot_id`.

#### Key Responsibilities

| Responsibility | Details |
|---------------|---------|
| **Task Allocation** | Only the leader robot (`robot_id == 0`, `hb_crystal`) runs the allocator timer. Uses Hungarian Algorithm to optimally assign crates to idle robots based on Euclidean distance. |
| **Path Planning** | Theta\* planner generates any-angle paths through the obstacle field (other robots + crates + drop zones). Paths are sent to robots via `/task_assignments`. |
| **State Machine** | Each robot follows: `IDLE → NAVIGATE_TO_CRATE → PICK → TRANSPORT → PLACE → RETURN → IDLE` |
| **Zone Locking** | Distributed zone mutex via `/zone_status` topic prevents two robots from entering the same drop zone simultaneously. |
| **PID Control** | Independent X, Y, and θ PID controllers with anti-windup for smooth omnidirectional motion. |
| **Collision Avoidance** | Artificial Potential Fields (APF) repulsion from other robots and non-target crates, blended with PID attraction. |
| **Pyramid Stacking** | `DropZoneManager` tracks slot occupancy and computes correct (x, y, layer) placement for pyramid stacking. |

#### State Machine Flow

```
IDLE ──(task assigned)──► NAVIGATE_TO_CRATE
                               │
                    (IR triggered + aligned)
                               │
                               ▼
                             PICK
                               │
                        (gripper attached)
                               │
                               ▼
                           TRANSPORT
                               │
                       (at drop slot + zone locked)
                               │
                               ▼
                            PLACE
                               │
                       (arm lowered + gripper released)
                               │
                               ▼
                            IDLE ──(5s timeout)──► RETURN ──► IDLE
```

---

### 2. 👁️ `PoseDetector` (`perception.py`)

Runs as the `localization_node`. Uses an overhead camera and ArUco markers for full-arena localization.

#### Key Features

- **Homography-based world mapping**: Detects 4 corner markers (IDs 1, 3, 5, 7) to compute a pixel→world homography matrix dynamically each frame.
- **Parallax Correction**: Compensates for apparent position shift of elevated crate markers (60 mm height) relative to the camera.
- **CLAHE Enhancement**: Adaptive histogram equalization improves detection in uneven lighting.
- **Published Data**:
  - `/bot_pose` — Positions and orientations of all 3 robots
  - `/crate_pose` — Positions and orientations of all visible crates

#### ArUco Marker ID Map

| IDs | Role |
|-----|------|
| `1, 3, 5, 7` | Arena corner markers (homography anchors) |
| `0, 2, 4` | Robot body markers |
| `10–99` | Crate markers |

---

### 3. 🌉 `MqttBridge` (`mqtt_bridge_node.py`)

Bridges ROS 2 commands to the ESP32 over WiFi MQTT.

#### Functionality

| Feature | Details |
|---------|---------|
| **Subscribes** | `/bot_cmd` (BotCmdArray) — receives motor + servo + solenoid commands |
| **Publishes (MQTT)** | `esp/cmd/1` — JSON payload sent to ESP32 |
| **Service Server** | `/attach` (SetBool) — sets solenoid state; the state is merged into the next motor command |
| **Subscribes (MQTT)** | `esp/sensor/1` — receives IR sensor readings from ESP32 |
| **Publishes** | `/ir_sensor_status` (Bool) — True if IR reads LOW (object detected) |

#### MQTT Payload Format

```json
{
  "m1": 150.0,
  "m2": -75.0,
  "m3": -75.0,
  "base": 175,
  "elbow": 100,
  "solenoid": 1
}
```

---

## ⚙️ Algorithms & Logic

### 🗺️ Theta\* Path Planner

An **any-angle A\*** variant that allows straight-line paths between non-adjacent grid nodes using Bresenham line-of-sight checks. This eliminates the "staircase" artifacts of standard grid A\*.

- **Grid Resolution:** 50 mm/cell
- **Connectivity:** 8-connected neighbors
- **Heuristic:** Euclidean distance
- **Iteration Limit:** 5,000 nodes (returns empty path on timeout to allow retry)

### 🏗️ Pyramid Stacking (DropZoneManager)

Crates are placed in a **pyramid pattern** within each drop zone. The algorithm:

1. Generates base-layer grid slots with fixed X/Y increments (60 mm × 70 mm)
2. Uses a diagonal sweep (sum `s = layer + slot_index`) to iterate pyramid coordinates
3. Each stacking layer shifts X position by `step_x / 2` and uses a higher arm angle

```
Layer 0 (Base):   [■] [■] [■]   ← Ground level
Layer 1:            [■] [■]     ← Stacked on top
Layer 2:              [■]       ← Apex
```

### 🎯 Task Allocation (Hungarian Algorithm)

The leader robot builds a **cost matrix** (N robots × M crates) where each cost is the Euclidean distance from robot to crate. `scipy.optimize.linear_sum_assignment` finds the globally optimal minimum-cost assignment.

### 🧲 Crate Pickup Sequence

1. **Approach**: Follow pre-planned Theta\* path to within 170 mm of crate
2. **Alignment**: PID theta controller aligns arm toward crate face
3. **IR Confirmation**: State transitions to `PICK` only when IR sensor triggers AND angle error < 9°
4. **Pick**:
   - Move arm to pickup position (base=175°, elbow=100°)
   - Activate solenoid via `/attach` service
   - Wait 4 seconds for magnetic attachment
   - Lift arm (base=120°, elbow=65°)
5. **Transport**: Navigate to drop slot with zone lock

### 🔒 Distributed Zone Locking

Robots broadcast zone occupancy on `/zone_status` in the format:
```
"D1,OCCUPIED,0"   →  Zone D1 locked by robot ID 0
"D1,FREE,-1"      →  Zone D1 released
```
Any robot waiting for a locked zone stops and polls until the zone is freed. The leader robot also acts as a **supervisor**, force-unlocking zones held by robots that have reported task completion.

---

## 📢 ROS 2 Topics & Services

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/bot_pose` | `Poses2D` | Perception → Controller | All robot positions and orientations |
| `/crate_pose` | `Poses2D` | Perception → Controller | All visible crate positions |
| `/bot_cmd` | `BotCmdArray` | Controller → Bridge | Motor + arm + solenoid commands |
| `/task_assignments` | `String` | Leader → All | Crate assignment + path + drop slot |
| `/task_complete` | `String` | All → Leader | Task completion reports |
| `/zone_status` | `String` | All ↔ All | Drop zone lock/unlock broadcasts |
| `/robot_{id}/ir_sensor_status` | `Bool` | Bridge → Controller | IR proximity sensor state |

### Services

| Service | Type | Server | Description |
|---------|------|--------|-------------|
| `/robot_{id}/attach` | `SetBool` | MQTT Bridge | Activate (True) / deactivate (False) solenoid gripper |

---

## 🔌 ESP32 Firmware

The ESP32 firmware runs on each robot and acts as the **hardware execution layer**.

### Responsibilities

- Connects to WiFi and MQTT broker on startup
- Subscribes to `esp/cmd/{id}` for motor + servo + solenoid commands
- Parses JSON payload and drives motors, servos, and solenoid accordingly
- Publishes IR sensor readings to `esp/sensor/{id}` every loop cycle
- Auto-reconnects on WiFi or MQTT disconnect

### MQTT Topics (ESP32 side)

| Topic | Direction | Content |
|-------|-----------|---------|
| `esp/cmd/1` | Subscribe | JSON: motor velocities, servo angles, solenoid state |
| `esp/sensor/1` | Publish | JSON: `{"ir": 0}` (0 = object detected) |

### Configuration (update before flashing)

```cpp
const char* ssid        = "YOUR_WIFI_SSID";
const char* password    = "YOUR_WIFI_PASSWORD";
const char* broker_ip   = "YOUR_BROKER_IP";   // Laptop running MQTT broker
const int   broker_port = 1883;
```

---

## 🚀 Setup & Installation

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Python dependencies
pip install opencv-python numpy scipy paho-mqtt

# Custom interfaces package (build separately)
colcon build --packages-select hb_interfaces
```

### MQTT Broker (Mosquitto)

```bash
sudo apt install mosquitto
sudo systemctl start mosquitto
# Verify broker IP and update BROKER_IP in mqtt_bridge_node.py
```

### ESP32 Arduino Libraries

- `WiFi.h` (built-in)
- `PubSubClient` (install via Arduino Library Manager)

---

## ▶️ Running the System

Launch nodes in the following order:

```bash
# 1. Start MQTT broker (terminal 1)
mosquitto -v

# 2. Launch perception node (terminal 2)
ros2 run holo_battalion perception

# 3. Launch MQTT bridge (terminal 3)
ros2 run holo_battalion mqtt_bridge_node

# 4. Launch controller for each robot (terminals 4, 5, 6)
ros2 run holo_battalion controller --ros-args -p robot_name:=hb_crystal   -p robot_id:=0
ros2 run holo_battalion controller --ros-args -p robot_name:=hb_glacio    -p robot_id:=4
ros2 run holo_battalion controller --ros-args -p robot_name:=hb_frostbite -p robot_id:=2
```

> ⚠️ **Important:** Flash ESP32 firmware and ensure all robots are connected to the same WiFi network as the MQTT broker before launching.

---

## ✅ Success Criteria

| # | Criterion | Status |
|---|-----------|--------|
| 1 | ✅ Crate successfully picked from pickup zone | Verified via IR + gripper feedback |
| 2 | ✅ Crate placed in correct color-coded zone | Determined by ID % 3 rule |
| 3 | ✅ Robot returned to docking position | Post-placement RETURN state |
| 4 | ✅ Zero robot-to-crate collisions | APF repulsion + path planner inflation |
| 5 | ✅ Continuous video recording without interruption | Overhead camera with CLAHE |
| 6 | ✅ Multi-robot coordination without zone conflicts | Distributed zone locking |
| 7 | ✅ Pyramid stacking | Layer-aware slot allocation |

---

## ⚠️ Known Constraints

1. **Arena Boundaries**: All robot movements must stay within the 2438.4 mm × 2438.4 mm arena. An emergency stop triggers if the robot approaches within 50 mm of any boundary.
2. **No Cuts in Video**: Recorded video must be continuous with no edits or lag.
3. **Collision Avoidance**: Robots must not collide with crates during pickup or delivery — enforced by planner inflation and APF repulsion fields.
4. **Path Retry Limit**: Theta\* is capped at 5,000 node expansions. If a path cannot be found (blocked), the robot waits 2.5 seconds and retries.
5. **Zone Locking Latency**: Due to ROS pub/sub asynchrony, a single loop cycle is skipped after locking a zone to allow the state to propagate before the robot begins moving.

---

## 📜 License

This project is developed as part of the **e-Yantra Robotics Competition (eYRC)** by IIT Bombay. All code is the original work of Team **HB_2442**.

---

<div align="center">

Made with ❤️ by Team HB_2442 &nbsp;|&nbsp; eYRC Holo Battalion

</div>
