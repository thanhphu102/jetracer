# JetRacer Autonomous MVP

This package implements the `plan.md` single-node MVP:

- OpenCV line following and cross-intersection detection.
- YOLO perception for signs, lights, and avoidance labels.
- A filtered perception layer.
- A public state machine with pending sign actions.
- A controller with steering smoothing, turn phases, and safe stops.

## Run

```bash
roslaunch jetracer_autonomous autonomous_drive.launch
```

The launch file passes an absolute config path using `$(find jetracer_autonomous)`.

## ROS Host + YOLO Docker

If ROS runs on the JetRacer host and YOLO runs in a Docker container, set:

```yaml
model:
  backend: "http"
  http_url: "http://127.0.0.1:8765/detect"
```

Start the YOLO container with host networking and mount this repo:

```bash
docker run -it --rm \
  --network host \
  --runtime nvidia \
  -v ~/catkin_ws/src/jetracer:/workspace/jetracer \
  your_ultralytics_image:latest \
  bash
```

Inside the container:

```bash
cd /workspace/jetracer
python3 jetracer_autonomous/tools/yolo_http_service.py \
  --model /workspace/jetracer/jetracer_autonomous/models/best.pt \
  --host 0.0.0.0 \
  --port 8765 \
  --conf 0.6
```

On the host, check the service:

```bash
curl http://127.0.0.1:8765/health
```

Then launch the ROS node on the host as usual.

To run the host ROS camera, YOLO Docker service, and autonomous node from one shell:

```bash
~/catkin_ws/src/jetracer/jetracer_autonomous/tools/run_host_ros_yolo_docker.sh
```

Useful overrides:

```bash
YOLO_PORT=8765 YOLO_CONF=0.6 YOLO_DEVICE=0 \
REPO_PATH=~/catkin_ws/src/jetracer \
~/catkin_ws/src/jetracer/jetracer_autonomous/tools/run_host_ros_yolo_docker.sh
```

Check whether YOLO can see CUDA/GPU:

```bash
curl http://127.0.0.1:8765/health
```

Open `rqt_image_view` and inspect:

- `/camera/image_raw` for the raw camera.
- `/jetracer_autonomous_drive/debug_overlay` for ROI, line center, state, detections.
- `/jetracer_autonomous_drive/line_mask` for the threshold mask used by line following.

## Safe First-Run Checklist

### Step 1 - Dry Run

Set:

```yaml
debug:
  dry_run: true
```

Check logs and overlay before allowing the vehicle to move:

- line error is reasonable.
- cross detection appears only near real intersections.
- YOLO detections are stable.
- state transitions match the track.
- `yolo_ran` appears at the expected interval.
- `internal_phase` appears during turns.
- `saved_state` appears during `AVOID` and `SLOW_DOWN`.
- `SIGN_PENDING` timeout works.
- steering and throttle values look safe.

### Step 2 - Wheels Off Ground

Place the vehicle on a stand and use very low throttle:

```yaml
throttle:
  normal: 0.08
  slow: 0.06
  turn: 0.05

debug:
  dry_run: false
```

Check steering direction, wheel direction, and zero-command stop behavior.

### Step 3 - Floor Test at Minimum Speed

Use low throttle:

```yaml
throttle:
  normal: 0.10
  slow: 0.07
  turn: 0.06
```

Test in this order:

1. line following only
2. cross detection only
3. sign pending only
4. `SIGN_PENDING` timeout
5. turn only
6. slow-down behavior
7. avoidance behavior
8. full flow

Keep an emergency stop ready.
