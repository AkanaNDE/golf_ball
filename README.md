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
