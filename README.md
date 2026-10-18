# golf_ws: golf ball collector robot

4-wheel skid-steer (diff drive) robot, ESP32 + micro-ROS, Xbox controller teleop. ROS 2 Jazzy.

```
Xbox ──USB──> joy_node ──/joy──> xbox_teleop ──/cmd_vel─────────────────┐
                                                                         │ micro_ros_agent (serial)
wheel_odometry <──/wheel_vel── ESP32 (kinematics + PID + motors) <───────┘
      └──> /odom + TF odom->base_link
```

## Layout
| Path | What |
|---|---|
| `src/golf_bot_control` | `xbox_teleop` (joy → cmd_vel) and `wheel_odometry` (wheel_vel → odom) |
| `src/golf_bot_vision` | `golf_ball_detector` (camera + YOLO11n `models/golf_ball_yolo11n.pt`) and `ball_chaser` (drive to nearest ball) |
| `src/golf_bot_tags` | `tag_mission`: AprilTag scan + time-optimal tour (see below), `tag_focal_calibration` |
| `src/golf_bot_bringup` | `bringup.launch.py` and `config/golf_bot.yaml` (all tunable parameters) |
| `firmware/golf_bot_esp32` | PlatformIO micro-ROS firmware, pins and dimensions in `include/config.h` |

## Build
```bash
cd ~/golf_ws && colcon build --symlink-install
source ~/golf_ws/install/setup.bash
```
Firmware: micro_ros_platformio fails (`No 'rosidl_typesupport_cpp' found`) if ROS 2 is sourced
in the shell, and `~/.bashrc` sources it. Build/upload from a clean environment:
```bash
cd ~/golf_ws/firmware/golf_bot_esp32 && env -i HOME=$HOME PATH=/usr/bin:/bin bash -c '~/.platformio/penv/bin/pio run -t upload'
```
If a build already failed that way, run `pio run -t clean_microros` (same clean env) first.

## Run
```bash
ros2 launch golf_bot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
```
Test the controller without the robot: `use_agent:=false`, then `ros2 topic echo /cmd_vel`.

## Xbox controls
| Input | Action |
|---|---|
| LB (hold) | Deadman: robot only moves while held |
| Left stick ↑↓ | Forward / backward |
| Right stick ←→ | Turn |
| RB (hold) | Turbo |
| Y | AUTO on/off: chase golf balls (needs `use_vision:=true`) |
| B | Emergency stop (latched, also turns AUTO off) |
| Start | Release emergency stop |

## 4-wheel diff drive equations
`v` = linear.x (m/s), `ω` = angular.z (rad/s), `W` = TRACK_WIDTH, `r` = WHEEL_RADIUS

Inverse (ESP32, `controlStep()`): both wheels on a side turn at the same speed
```
ω_FL = ω_RL = (v − ω·W/2) / r
ω_FR = ω_RR = (v + ω·W/2) / r
```
Forward (`wheel_odometry`):
```
v = r·(ω_R + ω_L)/2        ω = r·(ω_R − ω_L)/W
```
For skid steer the wheels slip sideways when turning, so `W` is an *effective* track width
(usually larger than the measured one): calibrate it, see below.

## Safety layers
1. Teleop only drives while LB is held; stops if `/joy` is silent for 0.5 s.
2. ESP32 stops wheels if `/cmd_vel` is older than 500 ms.
3. ESP32 stops everything if the micro-ROS agent connection is lost.

## Hardware (schematic V1.0, Drive System)
ESP32 DevKit, 2x Cytron MDD10A, 4x 36GP-555 with magnetic encoder, 24V Li-ion.

| Motor | Corner | Driver | PWM | DIR | Enc A | Enc B |
|---|---|---|---|---|---|---|
| M1 | front left  | D1 ch1 | 16 | 15 | 13 | 12 |
| M2 | front right | D1 ch2 | 17 | 18 | 14 | 27 |
| M3 | rear left   | D2 ch1 | 19 | 21 | 26 | 25 |
| M4 | rear right  | D2 ch2 | 23 | 22 | 33 | 32 |

⚠️ GPIO12 (P1B) is a boot strapping pin: if the encoder holds it HIGH at power-on the ESP32
will not boot (flash voltage set to 1.8 V). Fix either by moving P1B to free GPIO4/GPIO5
(and changing `M1_CONFIG`), or by burning the efuse once:
`espefuse.py --port /dev/ttyUSB0 set_flash_voltage 3.3V`.

## Motor test (no ROS)
Lift the wheels, then:
```bash
cd ~/golf_ws/firmware/golf_bot_esp32 && env -i HOME=$HOME PATH=/usr/bin:/bin bash -c '~/.platformio/penv/bin/pio run -e motor_test -t upload && ~/.platformio/penv/bin/pio device monitor'
```
`1`-`4` spin one motor, `f` all forward, `m` full-speed test, `c` clear counts, `+`/`-` duty, `s` stop.
Afterwards upload the real firmware again (`-e esp32dev`).

## Tuning checklist
1. `config.h`: `WHEEL_RADIUS`, `TRACK_WIDTH`, `WHEEL_FL/FR/RL/RR` corner mapping.
2. motor_test `1`-`4`: each wheel must spin forward and its count must go **up**.
   Flip `invert_motor` / `invert_encoder` in `M*_CONFIG` if not.
3. motor_test `c`, turn one wheel exactly 1 revolution by hand: put that count in
   `ENCODER_TICKS_PER_REV`.
4. motor_test `m`: rad/s at 100% → `MAX_WHEEL_SPEED`; then tune `PID_KP` / `PID_KI`.
5. Calibrate `TRACK_WIDTH` (skid steer): spin 360° in place, scale until `/odom` yaw matches.
   Copy the same `WHEEL_RADIUS` / `TRACK_WIDTH` into `config/golf_bot.yaml`.

## Raspberry Pi 5 (Ubuntu 24.04 arm64)
One-time setup (ROS 2 Jazzy, micro-ROS agent, build, serial permission, `~/.bashrc`):
```bash
git clone https://github.com/AkanaNDE/golf_ball.git ~/golf_ws
bash ~/golf_ws/scripts/setup_pi.sh
```
Log out and back in, then `ros2 launch golf_bot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0`.

Update after pushing changes from the PC:
```bash
cd ~/golf_ws && git pull && colcon build --symlink-install
```
Flash the ESP32 from the PC (PlatformIO); the Pi only needs the USB cable to the ESP32.

## Golf ball detection and AUTO mode
```bash
pip install ultralytics      # once (on Ubuntu 24.04 add --break-system-packages)
ros2 launch golf_bot_bringup bringup.launch.py use_vision:=true
```
Press **Y** to start AUTO: `ball_chaser` turns toward the nearest ball, drives to it, slows down
as it gets close, drives straight over it (`collect_time`), then searches for the next one.
Holding **LB** takes over manually at any time; **B** stops everything; AUTO also stops if the
joystick or the detector goes silent. View the camera: `ros2 run rqt_image_view rqt_image_view /golf_ball/image`.

| Topic | Type | Meaning |
|---|---|---|
| `/golf_ball/target` | PointStamped | x = offset −1 (left) .. +1 (right), y = ball bottom 0 (top) .. 1 (bottom/close), z = confidence |
| `/golf_ball/image` | Image | annotated camera image |
| `/cmd_vel_auto` | Twist | ball_chaser command, forwarded by xbox_teleop only in AUTO |
| `/ball_chaser/state` | String | SEARCH / TRACK / WAIT / COLLECT |

Tuning (in `golf_bot.yaml`): `arrive_y` / `collect_time` so the ball ends up in the collector,
`kp_angular` if it oscillates (lower) or turns too slowly (higher), `max_linear` for speed.

Raspberry Pi 5: export the model to NCNN (about 4x faster on ARM) and point the detector at it:
```bash
cd ~/golf_ws/src/golf_bot_vision/models && yolo export model=golf_ball_yolo11n.pt format=ncnn imgsz=320
```
then set `model: /home/<user>/golf_ws/src/golf_bot_vision/models/golf_ball_yolo11n_ncnn_model` and `imgsz: 320`.

## View the Pi camera on the laptop
The Pi publishes `/golf_ball/image/compressed` (JPEG, ~12 KB/frame). On the laptop:
```bash
ros2 run golf_bot_vision image_viewer
```
Both machines need the same `ROS_DOMAIN_ID`. On a phone hotspot (multicast blocked) also set
each machine's peer in `~/.bashrc`, then open a new terminal:
```bash
export ROS_STATIC_PEERS=<IP of the other machine>   # laptop -> Pi IP, Pi -> laptop IP
```
Check with `ros2 topic list`: the laptop should list `/golf_ball/image/compressed`.

## AprilTag tour mission
Same robot, new task: turn 360° to find every AprilTag, measure each tag's distance from its
apparent size, then visit all of them in the fastest order.
```bash
ros2 launch golf_bot_bringup tag_mission.launch.py                          # real robot, press Y
ros2 launch golf_bot_bringup tag_mission.launch.py simulate:=true use_agent:=false   # dry run
ros2 run golf_bot_vision image_viewer --ros-args -p topic:=tag_mission/image/compressed
```
Without joystick (starts by itself after `start_delay`, default 5 s):
```bash
ros2 launch golf_bot_bringup tag_mission_auto.launch.py
ros2 topic pub --once /tag_mission/stop std_msgs/msg/Bool "{data: true}"    # pause (false = resume)
```
Parameters: `src/golf_bot_bringup/config/tag_mission.yaml` (set `tag_size` to your printed tags).

**Algorithm: Scan → Calibrate → Map → Plan → Visit → Relocalize → Replan**

| Step | What happens |
|---|---|
| Scan | Turn in place ~405° in **30° steps: turn → stop → wait 0.5 s → look 1 s**. The camera runs a **short manual exposure** (`exposure_ms: 5`): blur = turn rate × exposure × focal, so at 0.4 rad/s auto exposure (~33 ms) smears a tag over ~9 px and it is lost, 5 ms gives ~1.3 px and tags are also detected **while turning** (their yaw is interpolated between the visually measured stops). |
| Visual yaw | The real turn between two stops is measured from the images: ORB features are matched and the median change of their bearing `atan((u−cx)/f)` is the turn. The scan stops after 360°+ of **real** rotation even if the wheels slip (the robot turned 225° while odometry said 526°) or stall; a stalled step raises the turn command, and the learned real/commanded ratio and minimum turn rate are used for the rest of the mission. |
| Turn tracking | Scan steps are **open loop by default: turn for `scan_step_time` (2.5 s ≈ 30° on this robot)**, then the real angle is measured at the stop. Optional (`turn_track: true`): every turn closed-loop on the camera: while turning, the current frame is matched against the frame from the start of the turn (640 px copy, ~7 Hz) and the turn stops when the measured angle is reached, minus a learned coasting lead. The heading is then set from the measurement. A turn the camera says is not moving raises the turn command. |
| Calibrate | Visual loop closure: a tag seen at stop A is seen again one *real* revolution later at stop B, so `scale = (360° + bearing_A − bearing_B) / turn(A→B)` fine-tunes the visual yaw (or calibrates odometry if the scene has no texture). (`scan_mode: continuous` uses centre crossings instead.) |
| Map | Distance `Z = f·tag_size / h_px` with `h_px` = mean of the two *vertical* edges (unchanged when the tag is turned). Observations are fused per tag with inverse-variance weights (σ ∝ Z/h_px, more at image edges); a tag needs ≥3 detections. |
| Plan | Minimise *time*, not just distance: leg cost = turn-in-place time + drive time + overhead, and the turn depends on the arrival heading, so the state is (visited set, previous tag, last tag). Held-Karp gives the exact optimum up to 10 tags (ms); insertion + 2-opt/or-opt above. Open tour (no return). |
| Visit | **Align** first: stand still → keyframe → turn up to 45° → stand still → measure the real turn from the images → correct the heading, until facing the tag or seeing it (dead-reckoned turns go wrong because the motor response is not linear). Then **approach stop-and-go**: stand still and look (thorough detection on sharp frames) → turn toward the tag or drive ≤ `approach_step` (0.4 m) → look again, until `standoff` in front (frames taken while driving are motion-blurred: 3 px of blur already hides a 4 cm tag at 1 m, at any resolution). After the scan the tag positions are **remembered** and the robot never sweeps around searching: it drives straight toward the remembered position (aimed by the camera if the tag is already visible) to `confirm_distance` before the standoff point, looks once, and finishes camera-guided if the tag is seen, otherwise by the remembered position (reported as "not seen on arrival"). |
| Relocalize | At each tag the measured offset to its mapped position corrects the robot's x/y, so drift does not accumulate. |
| Replan | Re-plan the remaining tags from the corrected pose after every visit / skip. |

Focal length for distances: put a tag 1.00 m in front of the camera and run
`ros2 run golf_bot_tags tag_focal_calibration --distance 1.0 --size 0.15` on the robot; Ctrl+C when
the printed value is stable, copy `focal_px` into the YAML.

**Camera on the laptop** (calibration, tag mission and golf ball mode alike): run
`ros2 run golf_bot_vision image_viewer` on the laptop; it shows whichever stream is active
(`/tag_calibration/...`, `/tag_mission/...`, `/golf_ball/image/compressed`, JPEG ~8 FPS).
Needs the same `ROS_DOMAIN_ID` and, on a phone hotspot, `ROS_STATIC_PEERS` (see above).
Tests: `cd src/golf_bot_tags && python3 -m pytest test/`.
