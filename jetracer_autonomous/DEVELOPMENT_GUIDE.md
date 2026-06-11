# JetRacer Autonomous: Development Guide

**High-Level Overview for Building the Autonomous Driving System**

This guide explains the **architecture and design decisions** behind the JetRacer Autonomous MVP, without extensive code examples.

> **Prerequisites:** Jetson Nano + ROS installed, Python 3.8+, dependencies installed

---

## 1. System Architecture

### The Three Layers

```
PERCEPTION LAYER
├── Camera Input
├── Line Detector (OpenCV HSV filtering)
├── YOLO Detector (signs, lights, obstacles)
└── Perception Filter (smooth & fuse)
        ↓
        Input: Raw perception data
        Output: Stable, filtered perception state

DECISION LAYER
├── State Machine (FOLLOWING_LINE, TURNING, STOPPING, ERROR)
├── Priority-based logic (safety > navigation > comfort)
└── Action output (go_straight, turn_left, turn_right, stop)
        ↓
        Input: Filtered perception
        Output: High-level decision/action

CONTROL LAYER
├── PID Controller (convert line offset to steering)
├── Steering Smoother (prevent jerky movements)
└── Speed Management (base_speed, turn_speed, stop_speed)
        ↓
        Input: Decision action + current line offset
        Output: Steering angle (°), Throttle (0-1)

VEHICLE INTERFACE
├── ROS Publisher (/cmd_vel)
└── Actuator commands to motor driver
        ↓
        Motors → Car moves
```

### Why This Design?

| Aspect | Reason |
|--------|--------|
| **Three-layer separation** | Each layer is independent; easy to test and swap components |
| **Perception → Decision → Control** | Mirrors decision-making pipeline: observe → decide → act |
| **State machine for decisions** | Clear logic, prevents invalid transitions, debug-friendly |
| **PID for steering** | Smooth, responsive control; tunable gains |
| **ROS-based** | Modular, scalable, industry standard for robotics |

---

## 2. Project Structure

### Directory Layout

```
jetracer_autonomous/
├── config/params.yaml              # All tunable parameters
├── models/best.pt                  # YOLO weights
├── launch/autonomous_drive.launch  # ROS launch file
├── scripts/
│   └── autonomous_drive_node.py    # Entry point (shebang script)
└── src/jetracer_autonomous/
    ├── __init__.py
    ├── config.py                   # Load YAML config
    ├── main_node.py                # Orchestrator
    ├── perception/
    │   ├── line_detector.py        # OpenCV line detection
    │   ├── yolo_detector.py        # YOLO inference
    │   └── perception_filter.py    # Temporal filtering
    ├── decision/
    │   └── decision_manager.py     # State machine
    ├── control/
    │   ├── steering_utils.py       # PID + Smoother
    │   └── controller.py           # Main control logic
    ├── vehicle/
    │   └── vehicle_interface.py    # ROS publisher
    └── utils/
        ├── logger.py               # Structured logging
        └── debug_overlay.py        # Visualization (optional)
```

### File Organization Principle

- **Config everything:** Parametrize heuristics, thresholds, gains in `params.yaml`
- **One class per module:** Each .py file contains one main class
- **Clear naming:** `LineDetector`, not `LD`; `perception_filter`, not `pf`
- **Type hints:** Python 3.8+ supports them; use them

---

## 3. Perception Layer

### Line Detector

**Purpose:** Find the center line on the track

**Algorithm:**
1. Convert BGR frame to HSV color space (robust to lighting)
2. Create binary mask for pixels in HSV range (from `params.yaml`)
3. Find largest contour (most likely the line)
4. Calculate center of mass
5. Return offset from frame center: negative = left, positive = right

**Input:** Camera frame (numpy array, BGR)  
**Output:** Line offset in pixels, or None if not detected

**Tuning knobs:**
- `hue_lower`, `hue_upper` — target line hue range
- `value_lower`, `value_upper` — brightness range (use high values for bright lines)

---

### YOLO Detector

**Purpose:** Recognize traffic signs, lights, obstacles

**Two backends:**
- **Local:** Load model with ultralytics library; runs on Nano GPU
- **HTTP:** Send frames to remote Docker container; runs on host machine (parallel)

**Input:** Camera frame  
**Output:** List of detections `[{class, confidence, x, y, width, height}, ...]`

**Tuning knobs:**
- `confidence_threshold` — minimum confidence to report a detection
- `iou_threshold` — non-maximum suppression (remove overlapping detections)

**Why two backends?**
- Local: fast, self-contained, lower power
- HTTP: offload compute to host GPU, prevents frame drops

---

### Perception Filter

**Purpose:** Reduce false positives and smooth detections across time

**Strategy:** Temporal filtering using sliding window history

**For line detection:**
- Maintain history of last N line offsets
- Return moving average (or None if insufficient history)
- Smooths jitter due to shadows, lighting changes

**For object detection:**
- Track which object classes appear consistently across frames
- Keep only detections seen in 50%+ of frames
- Eliminates one-frame false positives

**Design benefit:** Decouples detector confidence from filter robustness; a weak detector can become robust with filtering

---

## 4. Decision Layer

### State Machine

**States:**
- `FOLLOWING_LINE` — Normal operation, track center line
- `TURNING_LEFT` / `TURNING_RIGHT` — Executing a turn (with timeout)
- `STOPPING` — Stop immediately (red light, obstacle, line lost)
- `ERROR` — Unrecoverable error condition

**Transitions:**

```
FOLLOWING_LINE
├─ [Line lost] → STOPPING
├─ [Red light detected] → STOPPING
├─ [Obstacle detected] → STOPPING
├─ [Left arrow detected] → TURNING_LEFT
└─ [Right arrow detected] → TURNING_RIGHT

TURNING_LEFT/RIGHT
├─ [Turn timeout reached] → FOLLOWING_LINE
└─ [Red light detected] → STOPPING

STOPPING → FOLLOWING_LINE (when line redetected)
```

### Decision Logic Priority

```
1. Safety first: Red light? Obstacle? → STOP
2. Navigation: Sign detected? → TURN
3. Default: Line visible? → FOLLOW
4. Failsafe: Can't decide → STOP
```

**Why this order?** Safety overrides navigation overrides comfort

### Turn Timeout

**Question:** What if the car keeps detecting the left-arrow sign? It would turn forever!

**Answer:** Add a timeout (e.g., 2 seconds); after timeout, revert to `FOLLOWING_LINE` and assume we've completed the turn.

---

## 5. Control Layer

### PID Controller

**Purpose:** Convert line offset (error) into smooth steering adjustment

**Formula:** 
```
steering = kp * error + ki * integral(error) + kd * derivative(error)
```

**Terms:**
- **P (Proportional):** Main response; 80% of steering
- **I (Integral):** Eliminate steady-state error; rarely needed (usually ki=0)
- **D (Derivative):** Damping, prevent oscillation; add smoothness

**Typical gains for line following:**
- `kp = 0.1` — steering angle per pixel of offset
- `ki = 0.0` — skip it for simplicity
- `kd = 0.05` — damping term

**Tuning:**
- **Too much kp:** Jerky, overshoots turns
- **Too little kp:** Sluggish, doesn't correct enough
- **kd helps:** Smooths out oscillation from high kp

---

### Steering Smoother

**Purpose:** Prevent abrupt steering changes that could cause skids

**Algorithm:** Exponential moving average
```
smooth = alpha * new + (1 - alpha) * previous
```

**Interpretation:**
- `alpha = 1.0` — no smoothing (instant response)
- `alpha = 0.3` — heavy smoothing (sluggish)
- `alpha = 0.7` — moderate smoothing (responsive yet smooth)

**Design benefit:** Decouples control speed from perception noise; can smooth without affecting PID tuning

---

### Controller

**Responsibilities:**
1. Receive decision action (e.g., "turn_left")
2. Get current line offset
3. Compute steering angle:
   - If "go_straight": Use PID with line offset
   - If "turn_left": Fixed angle (-30°)
   - If "turn_right": Fixed angle (+30°)
   - If "stop": 0°
4. Compute throttle:
   - "go_straight": base_speed (e.g., 0.3)
   - "turn_left"/"turn_right": turn_speed (e.g., 0.25, slower to avoid tipping)
   - "stop": 0.0
5. Apply smoothing to steering
6. Clamp both values to safe ranges
7. Return (steering_angle, throttle) tuple

**Output:** Ready for ROS publication

---

## 6. Vehicle Interface & Main Node

### Vehicle Interface

**Responsibility:** Bridge between code and hardware

**Mechanism:**
- Create ROS Publisher for `/cmd_vel` topic
- Publish geometry_msgs/Twist message
- Mapping:
  - `msg.linear.x` ← throttle (0-1)
  - `msg.angular.z` ← steering angle (normalized -1 to +1)

---

### Main Node

**Role:** Orchestrator that ties everything together

**Workflow each frame:**
```
1. Receive camera frame via ROS subscription
2. Run line detector → raw line offset
3. Run YOLO → raw detections
4. Filter both raw outputs
5. Decision manager decides action
6. Controller computes steering + throttle
7. Vehicle interface publishes ROS command
8. Log everything for debugging
9. If error → immediate stop for safety
```

**Frame rate:** Must complete all steps within ~33ms for 30 FPS camera

---

## 7. Configuration

### params.yaml Structure

```yaml
line:                            # Line detection HSV ranges
  hue_lower: 0
  hue_upper: 180
  value_lower: 200
  value_upper: 255

model:                           # YOLO configuration
  backend: "local"               # or "http" for Docker
  model_path: "models/best.pt"
  confidence_threshold: 0.6
  iou_threshold: 0.45

control:                         # Motor control parameters
  max_steering_angle: 30
  steering_smoothing_factor: 0.7
  base_speed: 0.3
  turn_speed: 0.25

decision:                        # State machine tuning
  turn_timeout: 2.0
```

**Philosophy:** Every parameter is tunable; no magic numbers hardcoded in source

---

## 8. Development Workflow

### Step 1: Test Perception Independently

1. Capture frames from camera
2. Run line detector; visualize mask
3. Run YOLO detector; check detections
4. Verify filter is smoothing correctly

**Tools:** OpenCV imshow, rqt, rostopic echo

### Step 2: Test Decision Logic

1. Mock various perception states (line offset = 50px, line offset = None, etc.)
2. Feed into decision manager
3. Verify state transitions are correct

**Tools:** Unit tests, print statements, state logs

### Step 3: Test Control Loop

1. Mock various actions ("go_straight", "turn_left", etc.)
2. Feed into controller with various line offsets
3. Observe steering + throttle outputs
4. Verify PID gains are reasonable (not jerky)

**Tools:** Simulation scripts, plot steering vs line offset

### Step 4: Integration Test

1. Run full launch file
2. Place car on track
3. Monitor logs and ROS topics
4. Adjust parameters based on behavior

---

## 9. Debugging Strategy

### When Line Detection Fails

- **Symptoms:** Car doesn't follow line or drifts off
- **Check:** Is track lighting good? Frame rate dropping?
- **Solution:** Adjust HSV ranges in params.yaml; test with known track images

### When YOLO Misses Signs

- **Symptoms:** Car doesn't respond to traffic signs
- **Check:** Is confidence threshold too high? Does model work on sample images?
- **Solution:** Lower threshold incrementally; check model accuracy

### When Steering is Jerky

- **Symptoms:** Car wobbles instead of smooth tracking
- **Check:** Are PID gains too high? Is smoother factor too low?
- **Solution:** Reduce kp or increase smoothing_factor

### When Car Overshoots Turns

- **Symptoms:** Car turns too much and drifts off line at intersections
- **Check:** Turn speed too high? turn_timeout too long?
- **Solution:** Lower turn_speed or reduce turn_timeout

### When Car Doesn't Receive Commands

- **Symptoms:** Motors don't respond; /cmd_vel not published
- **Check:** Are all nodes running? Is camera topic publishing?
- **Solution:** `rostopic list`, `rostopic hz`; check logs

---

## 10. Performance Tips

### Optimize for Real-Time

- Line detection: O(N) where N = pixels; fast
- YOLO: ~30-50ms per frame; use smaller model if too slow
- Filter: O(1) with fixed window size
- Decision: O(M) where M = detections; negligible
- Control: O(1); fast

### If Frame Rate Drops

1. **Use smaller YOLO model** — nano or small variant
2. **Increase inference interval** — run YOLO every 2nd frame
3. **Use HTTP backend** — offload to host GPU

### If Steering is Unstable

1. Increase smoothing factor (higher = smoother)
2. Reduce PID kp (lower = less responsive but stabler)
3. Check camera mount for vibration

---

## 11. Next Steps After MVP

- **Multi-sensor fusion:** Combine line + YOLO for robustness
- **Adaptive speeds:** Reduce speed on sharp turns
- **Advanced state logic:** Intersection detection + route history
- **Model retraining:** Collect data from your track, retrain YOLO
- **Hardware optimization:** Jetson Nano power limits, thermal throttling

---

## Summary

### Key Concepts

1. **Three-layer architecture:** Perception → Decision → Control
2. **State machine:** Clear, debuggable decision logic
3. **Temporal filtering:** Robust to sensor noise
4. **PID control:** Smooth, tunable steering
5. **ROS-based:** Modular, professional robotics standard

### Development Mindset

- **Parametrize:** Use config file, never hardcode
- **Log:** Log perception, decisions, control for debugging
- **Test independently:** Each layer can be unit tested
- **Iterate:** Start simple (line following only), then add features
- **Monitor:** Use ROS tools (rostopic, rosgraph, rqt) to inspect system

Good luck building! 🚗
