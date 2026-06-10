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
