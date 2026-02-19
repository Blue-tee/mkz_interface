mkz_interface

# To build mkz_interface package only.
  # Jagrat's build:
cd ~/autoware_mkz_ws
colcon build --merge-install --symlink-install --packages-select mkz_interface
colcon build --symlink-install --packages-select mkz_interface
  # Skylar's build:
cd /autoware_mkz/autoware_mkz_s
rm -rf build install log
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release   --packages-up-to mkz_interface

# Source and launch mkz_interface.
source install/setup.bash
source ~/autoware_mkz_ws/install/setup.bash
ros2 launch mkz_interface interface.launch.py

# Engage (if you gate on engage)
ros2 topic pub --once /vehicle_cmd_gate/output/engage autoware_vehicle_msgs/msg/Engage "{engage: true}"

ros2 service call /api/autoware/set/engage tier4_external_api_msgs/srv/Engage "{engage: true}"

# To turn steering wheel,brake,accel (test)
ros2 topic pub --rate 20 /vehicle_cmd_gate/output/command/control_cmd   autoware_control_msgs/msg/Control   "{lateral: {steering_tire_angle: 0.00}, longitudinal: {acceleration: 0.00}}"

# To use turn signals
ros2 topic pub /vehicle_cmd_gate/output/command/turn_indicators_cmd autoware_vehicle_msgs/msg/TurnIndicatorsCommand "{command: 3}"

# To change gears (D 2-19) (R 20,21) (P 1) (N 3)
ros2 topic pub --once /vehicle_cmd_gate/output/command/gear_cmd \
autoware_vehicle_msgs/msg/GearCommand "{command: 2}" 


ros2 interface show dbw_ford_msgs/msg/ThrottleCmd
ros2 interface show dbw_ford_msgs/msg/BrakeCmd

# Dataspeed start - from the Dataspeed launch guide
ros2 launch dbw_ford_can dbw.launch.xml
ros2 topic pub /vehicle/enable std_msgs/Empty "{}"
ros2 topic echo /vehicle/dbw_enabled


 ros2 topic pub --once /vehicle_cmd_gate/output/engage autoware_vehicle_msgs/msg/Engage "{engage: true}"
 
# Topic.yaml parameter explained for tuning purposes.

Got it. Below is a parameter-by-parameter “owner’s manual” for your MKZ interface, organized so you can tune it confidently.

There are two nodes and one shared params YAML:

* Node A: `autoware_to_dbw_can.py` (Autoware ➜ DBW commands)
* Node B: `dbw_can_to_autoware.py` (DBW reports ➜ Autoware status topics)
* `topics.yaml`: provides the parameter values (topic names, gains, limits, etc.)

A key rule for ROS2 params:

✅ If a param is set in `topics.yaml`, that value is used.
If it’s not in YAML, the `.py` default is used.
For the “drive profile” logic we fixed: explicit YAML values win over the profile.

---

# A) `autoware_to_dbw_can` parameters (Autoware → throttle/brake/steer/gear/misc)

## 1) Timing & safety

### `rate_hz` (default: 50.0)

What it does: Loop/publish rate for commands (throttle/brake/steer/misc).
Higher: smoother actuation, less pulsing; more CPU.
Lower: can feel “steppy”/jerky, especially brake and steering.

Reasonable range: 20–100 Hz
Typical: 50 Hz (good for DBW)

---

### `watchdog_ms` (default: 100)

What it does: If Autoware command messages are older than this, the node treats control as stale and stops commanding normally (safety behavior).
Lower: safer, but can cause pulsing if commands occasionally stall.
Higher: more tolerant of jitter; slower to fail-safe if Autoware dies.

Reasonable range: 150–600 ms for real-car tuning
Typical: 300 ms while tuning; can tighten later.

---

## 2) Topic wiring (you said these are correct)

These are just strings: changing them only changes what topics you subscribe/publish.

### Autoware inputs

* `aw_control_cmd`
* `aw_gear_cmd`
* `aw_turn_cmd`
* `aw_hazard_cmd`
* `aw_engage`

If wrong: node won’t receive commands (or receives the wrong ones).
No tuning “feel” effect besides whether data arrives.

### DBW outputs (commands you publish)

* `dbw_throttle_cmd`
* `dbw_brake_cmd`
* `dbw_steering_cmd`
* `dbw_gear_cmd`
* `dbw_misc_cmd`
* `dbw_enable_topic`
* `dbw_disable_topic`

If wrong: car won’t respond, or responds incorrectly.

---

## 3) Steering conversion & limits

### `steering_wheel_to_tire_ratio` (default: 14.8)

What it does: Converts Autoware tire angle command into steering wheel angle command for DBW.
Higher ratio: for the same tire angle, you command more wheel angle (more aggressive steering command).
Lower ratio: less wheel command for same tire angle (understeer / won’t turn enough).

Reasonable range: 12–20
Use: keep matching vehicle reality. If Autoware tracks path but wheel angle looks off, this is the knob.

---

### `max_steering_wheel_angle_deg` (default in YAML)

What it does: Saturation limit on wheel angle command.
Higher: allows tighter turns; risk of saturating steering / weird behavior at low speed.
Lower: protects system, but might fail sharp maneuvers.

Reasonable range: 400–600° (depends on DBW + vehicle)

---

### `steering_sign` (default in YAML)

What it does: flips steering direction (+1 or -1).
Wrong sign: car steers opposite.

Valid values: +1 or -1 only.

---

## 4) Steering “rate shaping” (this is your steering smoothness system)

These only matter if steering-velocity mode is enabled.

### `steer_vel_enable` (bool)

What it does: If enabled, the node limits how fast steering can change (reduces steering jerk).
Enable: smoother steering, less twitch.
Disable: more direct, can be sharp/jerky.

Typical: true on real car.

---

### `steer_vel_min`

What it does: minimum steering velocity floor (prevents “stuck” steering due to tiny dt/filters).
Higher: steering always moves at least a bit (can feel twitchy).
Lower: smoother but may feel sluggish around small corrections.

Reasonable: small positive number (you already have something sane in YAML)

---

### `steer_vel_gain`

What it does: how strongly steering velocity responds to steering error.
Higher: quicker steering response, potentially twitchy.
Lower: smoother, potentially laggy.

Reasonable: 0.5–3.0-ish depending on units in your implementation

---

### `steer_vel_tau` (seconds)

What it does: low-pass filter time constant for steering velocity.
Higher: more smoothing (less jerk), more delay.
Lower: snappier, more noise.

Reasonable: 0.1–0.6 s
Comfort: 0.3–0.5
Sport: 0.1–0.25

---

### `steer_vel_dv_max`

What it does: caps how fast steering velocity itself can change (jerk limiter for steering).
Higher: more immediate steering changes.
Lower: buttery steering but can feel delayed.

Reasonable: set so wheel doesn’t “snap” at engage.

---

### `steer_vel_cap_speeds`, `steer_vel_cap_vels`

What it does: speed-based cap on steering velocity (usually slower changes at higher speeds).
Higher caps: more responsive at speed, riskier.
Lower caps: stable on highway, slower lane changes.

Rule: always cap steering velocity more at higher speeds.

---

## 5) Longitudinal accel → throttle/brake mapping (base behavior)

These are the core gains. They decide how “powerful” throttle/brake responses are.

### `accel_to_throttle_gain` (k_th)

What it does: throttle% ≈ accel(m/s²) × k_th
Higher: more throttle for the same requested acceleration (snappier, can jerk).
Lower: gentler, might feel underpowered.

Reasonable: ~0.08–0.25
Start: 0.12–0.16 for comfort sedan feel.

---

### `accel_to_brake_gain` (k_br)

What it does: brake% ≈ (-accel) × k_br
Higher: stronger braking for same decel request (can feel grabby).
Lower: softer braking, can overshoot speed/stop.

Reasonable: ~0.15–0.40
Start: 0.22–0.30

---

### `max_throttle`

What it does: hard cap on throttle command.
Higher: more punch; can lurch at low speeds.
Lower: safer + smoother, but might not climb/accelerate.

Reasonable: 0.12–0.30
Comfort: 0.16–0.22
Sport: 0.22–0.30

---

### `max_brake`

What it does: hard cap on brake command.
Higher: can stop hard (safety), but can be abrupt.
Lower: smoother but might not stop quickly when needed.

Reasonable: 0.25–0.60
Typical: 0.35–0.45

---

### `throttle_deadband`, `brake_deadband`

What it does: below this value, command becomes zero (prevents tiny “buzzing”).
Higher deadband: less chatter, but creates a “nothing happens” zone.
Lower deadband: more precise but can oscillate.

Reasonable: 0.01–0.04
Typical: 0.02

---

# B) The “Smooth but Responsive” longitudinal shaping (new knobs)

These are the knobs that make it feel like a luxury MKZ instead of a robotic on/off controller.

### `lon_shaping_enable` (bool)

What it does: turns the smoothing logic on/off.

* true: uses filter + coast band + slew limits + stop-hold
* false: direct mapping (more jerky)

Always keep: true on real vehicle.

---

### `drive_profile` = `"comfort" | "normal" | "sport"`

What it does: sets good starting values for the knobs below.
Important: If you explicitly set any knob in YAML, your YAML wins.

Use:

* comfort: rain, demos, passenger comfort
* normal: daily tuning / best baseline
* sport: sharper response, still not jerky

---

## 1) Acceleration filtering (removes chatter)

### `accel_filter_tau` (seconds)

What it does: low-pass filters Autoware accel command before converting to throttle/brake.
Higher τ: smoother launch and speed hold, but more lag.
Lower τ: more responsive, but more jerk.

Reasonable: 0.10–0.40 s

* Comfort: 0.25–0.35
* Normal: 0.16–0.22
* Sport: 0.10–0.16

---

## 2) Coast band (kills accel↔brake flip-flopping)

### `accel_coast_band` (m/s²)

What it does: if |accel| is below this threshold, command coast (no throttle, no brake), unless stop-hold triggers.
Higher: less hunting around speed limit and near stop, more “gliding”.
Lower: more precise speed control, but can chatter.

Reasonable: 0.06–0.22

* Comfort: 0.14–0.20
* Normal: 0.10–0.14
* Sport: 0.06–0.10

---

## 3) Slew-rate limits (the “jerk limiter”)

These limit how fast throttle/brake can change.

### `thr_slew_per_s` (1/s)

What it does: max throttle change per second (0..1 scale).
Higher: more responsive, can jerk.
Lower: smoother, can feel lazy.

Reasonable: 0.4–1.5

* Comfort: 0.5–0.8
* Normal: 0.8–1.1
* Sport: 1.1–1.5

---

### `brk_slew_per_s` (1/s)

What it does: max brake change per second.
Usually brake can change a bit faster than throttle to stabilize speed without oscillation.

Reasonable: 0.6–2.0

* Comfort: 0.8–1.2
* Normal: 1.1–1.5
* Sport: 1.5–2.0

---

## 4) Stop-hold (fixes “2 mph then accel then slam stop”)

### `stop_hold_enable` (bool)

What it does: when you’re basically stopped, hold a small brake so the car doesn’t creep and cause oscillation.

Keep: true on real car.

---

### `stop_hold_speed_mps`

What it does: below this speed you’re considered “stopped.”
Higher: holds brake earlier (smoother stopping, less creep), may feel sticky.
Lower: more natural roll, but can creep/oscillate.

Reasonable: 0.10–0.35 m/s (≈ 0.2–0.8 mph)

---

### `stop_hold_accel_band`

What it does: only apply stop-hold when accel demand is small (so you can pull away normally).
Higher: holds more often (more stable), may resist inching.
Lower: holds less often (more natural), may allow creep.

Reasonable: 0.10–0.30 m/s²

---

### `stop_hold_brake` (0..1)

What it does: the brake percentage used for holding.
Higher: prevents creep strongly, can feel abrupt when it grabs.
Lower: gentle hold, may still creep.

Reasonable: 0.03–0.08 (3–8%)

---

# C) Gear change & safety behaviors (feel + correctness)

You’ve got gear-change brake-interlock logic (70% brake + hold, etc.). The key knobs (if present in your YAML/code) usually include things like:

* requiring DBW enabled / engaged before gear changes
* brake percent for gear shift
* pre/post hold times

Rule of thumb for luxury feel: keep brake hold smooth and consistent. Don’t “tap” brake.

If you want, paste your gear-change parameter block (or I can read it from the exact file you installed in your workspace), and I’ll document those knobs too.

---

# D) `dbw_can_to_autoware` parameters (DBW reports → Autoware status)

These are mostly topic wiring + simple conversions. Less “feel” tuning here.

### `steering_wheel_to_tire_ratio`

Same idea as above: used to compute tire angle status from steering wheel angle.

---

### DBW input topics

* `dbw_dbw_enabled`
* `dbw_steering_report`
* `dbw_gear_report`
* `dbw_misc_report`

If wrong, Autoware status will be wrong.

---

### Autoware output status topics

* `aw_velocity_status`
* `aw_steering_status`
* `aw_gear_status`
* `aw_turn_status`
* `aw_control_mode`
* `aw_hazard_status`

---

### `hazard_shim_topic`, `hazard_shim_timeout_ms`

What it does: some DBW stacks don’t report hazards exactly like Autoware expects; shim latches hazards briefly.
Higher timeout: hazard stays latched longer.
Lower timeout: hazard drops quickly.

Reasonable: 200–1500 ms

---

# “Which knobs do I touch first?”

If your goal is: smooth like a luxury sedan but still responsive, tune in this order:

1. `thr_slew_per_s`  (biggest jerk reduction without killing response)
2. `accel_filter_tau` (removes controller chatter)
3. `accel_coast_band` (kills accel↔brake flip at speed hold)
4. stop-hold knobs (fixes end-of-route behavior)
5. only then adjust `accel_to_throttle_gain` / `accel_to_brake_gain`

---

Then if it’s still jerky: reduce `thr_slew_per_s` or raise `accel_filter_tau`.

---




