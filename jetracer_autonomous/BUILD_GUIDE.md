# JetRacer Autonomous: Development & Build Guide

**High-Level Overview for Building the Autonomous Driving System from Scratch**

This guide teaches you how to **design, architect, and develop** the JetRacer Autonomous MVP. We assume you already have:
- ✅ Jetson Nano with OS installed
- ✅ ROS (melodic) configured
- ✅ Python 3.8+ 
- ✅ Required dependencies (numpy, opencv-python, ultralytics, PyYAML)
- ✅ Working camera and motor interface

**This guide focuses on:** System architecture, component roles, and how they integrate.

---

## Part 1: Understanding the System Architecture

### 1.1 The Data Pipeline

```
Camera Frame
    ↓
Perception (Line + YOLO Detection)
    ↓
Filtering (Remove noise, stabilize)
    ↓
Decision Manager (State machine: what to do?)
    ↓
Controller (Convert decision to steering/throttle)
    ↓
Vehicle Interface (Publish ROS commands)
    ↓
Motors → Car moves
```

### 1.2 Component Roles

| Component | Responsibility |
|-----------|-----------------|
| **Line Detector** | Use OpenCV HSV filtering to find the track center line |
| **YOLO Detector** | Recognize traffic signs, lights, obstacles |
| **Perception Filter** | Smooth detections across multiple frames (reduce false positives) |
| **Decision Manager** | State machine: make high-level decisions (go straight, turn, stop) |
| **Controller** | Convert decisions to steering angles (-30° to +30°) and throttle (0-1) |
| **Vehicle Interface** | ROS publisher that sends /cmd_vel messages to motor driver |

### 1.3 Key Principles

- **Modularity:** Each component is independent; communication via standardized data (dicts, floats)
- **Logging:** Log perception, decisions, and control commands for debugging
- **Safety:** Default to low speed, stop if line is lost
- **Real-time:** All operations must complete within ~33ms (for 30 FPS camera)

---

## Part 2: Project Structure & Setup

### 2.1 Create ROS Package Directory

```bash
cd ~/catkin_ws/src
catkin_create_pkg jetracer_autonomous rospy std_msgs geometry_msgs
cd jetracer_autonomous

# Create directory structure
mkdir -p src/jetracer_autonomous/{perception,control,decision,vehicle,utils}
mkdir -p config launch models
```

### 2.2 Configuration File

**`config/params.yaml`** contains all tunable parameters:

```yaml
line:
  hue_lower: 0
  hue_upper: 180
  value_lower: 200  # Only bright pixels
  value_upper: 255

model:
  backend: "local"   # or "http" for Docker
  model_path: "models/best.pt"
  confidence_threshold: 0.6

control:
  max_steering_angle: 30
  steering_smoothing_factor: 0.7
  base_speed: 0.3
  turn_speed: 0.25

decision:
  turn_timeout: 2.0
```

### 2.3 Dependencies

**`requirements.txt`:**
```
numpy>=1.19
opencv-python>=4.5
PyYAML>=5.3
ultralytics>=8.0
```

---

## Part 3: Perception Layer

The perception layer processes camera input to understand the environment.

### 3.1 Line Detector (`perception/line_detector.py`)

**Purpose:** Find the track center line using computer vision

**How it works:**
1. Convert camera frame from BGR to HSV color space (more robust to lighting)
2. Create a binary mask for pixels matching the line color range
3. Find contours and calculate the center of mass
4. Return offset from frame center (negative = left, positive = right)

**Input:** Camera frame (numpy array)  
**Output:** Line offset in pixels (or None if not detected)

**Key tuning:** HSV ranges in `params.yaml` - adjust `hue_lower/upper` and `value_lower/upper` based on your track lighting

---

### 3.2 YOLO Detector (`perception/yolo_detector.py`)

**Purpose:** Detect traffic signs, lights, obstacles

**Features:**
- **Local inference:** Load `best.pt` with ultralytics library (low latency, uses GPU)
- **HTTP inference:** Send frames to remote Docker container running YOLO (parallel processing)

**Input:** Camera frame  
**Output:** List of detections `[{class, confidence, x, y, width, height}, ...]`

**Key tuning:** `confidence_threshold` and `iou_threshold` in `params.yaml`

---

### 3.3 Perception Filter (`perception/perception_filter.py`)

**Purpose:** Reduce false positives by smoothing detections across multiple frames

**How it works:**
- Maintain a sliding window of past frames (default: 5 frames)
- For line detection: compute moving average of offsets
- For YOLO detections: keep only detections seen in 50%+ of frames

**Example:** If YOLO detects a "stop sign" for 1 frame then misses it, we filter it out. If it detects consistently, we trust it.

**Input:** Raw detections from one frame  
**Output:** Stable, filtered detections

---

## Part 4: Decision Layer

### Decision Manager (`decision/decision_manager.py`)

**Purpose:** State machine that decides what action to take based on perception

**States:**
- `FOLLOWING_LINE` — Normal operation, track the line
- `TURNING_LEFT/RIGHT` — Execute a turn (with timeout to prevent infinite turns)
- `STOPPING` — Stop immediately
- `ERROR` — Line lost or obstacle detected

**Decision Logic:**
1. **Safety first:** If red light or obstacle detected → STOP
2. **Check for sign commands:** If left/right arrow detected → TURN
3. **Normal operation:** If line detected → FOLLOW_LINE, else → STOP

**Key features:**
- Turn timeout (e.g., 2 seconds max per turn)
- State transitions logged for debugging
- Should not turn forever; always return to line following

**Input:** Filtered line offset, list of detections  
**Output:** Action string ("go_straight", "turn_left", "turn_right", "stop")

---

## Part 5: Control Layer

### PID Controller & Steering Smoother

**PID for line following:**
- **P (Proportional):** Steer proportional to line offset (larger offset → larger steering)
- **I (Integral):** Accumulate errors over time (helps eliminate constant drift)
- **D (Derivative):** React to rate of change (damping, prevents oscillation)

Typical gains: `kp=0.1, ki=0.0, kd=0.05`

**Exponential Smoother:**
- Prevents abrupt steering changes that could cause skids
- Formula: `smooth = alpha * new + (1-alpha) * previous`
- Lower alpha = smoother but sluggish; higher alpha = responsive but jerky

---

### Main Controller (`control/controller.py`)

**Purpose:** Convert decision actions into motor commands

**Mapping:**
| Action | Steering | Throttle |
|--------|----------|----------|
| `go_straight` | PID (depends on line offset) | base_speed (0.3) |
| `turn_left` | -max_angle (-30°) | turn_speed (0.25) |
| `turn_right` | +max_angle (+30°) | turn_speed (0.25) |
| `stop` | 0° | 0.0 |

**Process:**
1. Get action from decision manager
2. Calculate steering using PID for line offset
3. Apply exponential smoothing to steering
4. Clamp values to safe ranges
5. Return (steering_angle, throttle) tuple

**Input:** Action string, line offset (optional)  
**Output:** Steering angle in degrees, throttle in [0, 1]

---

## Part 6: Integration & Main Node

### Vehicle Interface (`vehicle/vehicle_interface.py`)

**Purpose:** Bridge between code and hardware

**Responsibilities:**
- Create ROS Publisher for `/cmd_vel` topic
- Convert steering angle + throttle into geometry_msgs/Twist message
- Send commands at regular intervals

**Mapping to ROS:**
- Linear velocity (x-axis) = throttle
- Angular velocity (z-axis) = steering angle (normalized)

---

### Config & Logger Utilities

**`config.py`:** Load `params.yaml` at startup

**`utils/logger.py`:** Log perception, decisions, and control for debugging

---

### Main Node (`main_node.py`)

**The orchestrator that ties everything together:**

```
1. Initialize all components (read params)
2. Subscribe to /camera/rgb/image_raw topic
3. For each frame:
   a. Run line detection
   b. Run YOLO detection
   c. Filter both detections
   d. Decision manager decides action
   e. Controller computes steering + throttle
   f. Vehicle interface sends ROS command
4. Log everything for debugging
```

**Error handling:** If any exception occurs, immediately call `vehicle_interface.stop()` for safety

---

### ROS Launch File (`launch/autonomous_drive.launch`)

Starts:
1. Camera driver node
2. Autonomous drive node
3. Passes config path as parameter

---

### Entry Point (`scripts/autonomous_drive_node.py`)

Simple shebang script that imports and calls `main()` from `main_node.py`

---

## Part 7: Build & Deployment

### Build the ROS Package

```bash
cd ~/catkin_ws
catkin_make --pkg jetracer_autonomous
```

### Run the System

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch jetracer_autonomous autonomous_drive.launch
```

### Monitor the System

In separate terminals:
```bash
# Check camera frame rate
rostopic hz /camera/rgb/image_raw

# Check command output
rostopic echo /cmd_vel

# Monitor logs
rosnode list
rqt
```

---

## Part 8: Testing & Debugging

### Test Components Independently

**Line detection:**
- Capture a frame from camera
- Run line detector
- Visualize the mask with OpenCV

**YOLO detection:**
- Test on sample images
- Check confidence scores match your model

**Decision manager:**
- Mock different perception states
- Verify state transitions

**Controller:**
- Test PID tuning (adjust kp, kd gains)
- Verify steering smoothing

### Performance Optimization

- **Slow inference:** Use smaller YOLO model or HTTP backend
- **Jittery steering:** Increase smoothing factor
- **Poor line detection:** Adjust HSV ranges in config
- **Overshooting turns:** Lower PID gains

---

## Summary: Workflow

1. **Understand the pipeline:** Camera → Perception → Decision → Control → Motors
2. **Set up project structure:** ROS package, directories, config file
3. **Implement perception:** Line detector + YOLO detector + filter
4. **Implement decision:** State machine based on perception
5. **Implement control:** PID controller + steering smoother
6. **Implement integration:** Main node orchestrates all components
7. **Build & test:** Build package, run launch file, debug issues
8. **Tune & optimize:** Adjust parameters based on performance

---

## Key Concepts

- **HSV color space:** Robust to lighting changes
- **Temporal filtering:** Use multiple frames to reduce false positives
- **State machine:** Always in one state; transitions based on perception
- **PID control:** Proportional to error; adds damping and integral terms
- **Smoothing:** Exponential filter prevents abrupt steering
- **ROS architecture:** Publish/subscribe for modular communication
- **Safety first:** Always default to stop; handle errors gracefully

---

## Next Steps

- Add **safety timeouts** (max time without line before emergency stop)
- Implement **multi-sensor fusion** (combine line + YOLO for robustness)
- Add **adaptive speed** (slower when turning, faster on straightaways)
- Create **telemetry dashboard** (live visualization of perception)
- Build **model training pipeline** (retrain YOLO on new data)

Good luck! 🚗
