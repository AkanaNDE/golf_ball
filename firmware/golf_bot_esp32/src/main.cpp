// Golf ball collector robot - ESP32 micro-ROS firmware
// Hardware: ESP32 DevKit + 2x Cytron MDD10A + 4x 36GP-555 motors with magnetic encoders
//
// Subscribes:
//   /cmd_vel    geometry_msgs/Twist   linear.x (m/s), angular.z (rad/s)
// Publishes:
//   /wheel_vel  geometry_msgs/Vector3 x = left, y = right wheel speed (rad/s, side average)
//
// 4-wheel differential drive (skid steer): see controlStep() for the equations.
// Each wheel has its own encoder and speed PID.
// Safety: wheels stop if /cmd_vel is stale or the micro-ROS agent is lost.

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "drive_hw.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This firmware expects board_microros_transport = serial
#endif

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
  } while (0)

// ---------------------------------------------------------------------------
// Speed control (per wheel)
// ---------------------------------------------------------------------------
struct WheelControl {
  float target = 0.0f;     // ramped target, rad/s
  float measured = 0.0f;   // filtered measured speed, rad/s
  float integral = 0.0f;
  int64_t last_count = 0;
};

WheelControl wheel_ctrl[4];

// Commands from ROS
volatile float cmd_linear = 0.0f;
volatile float cmd_angular = 0.0f;
volatile uint32_t last_cmd_ms = 0;

float rampTowards(float current, float goal, float max_step) {
  if (goal > current + max_step) return current + max_step;
  if (goal < current - max_step) return current - max_step;
  return goal;
}

// Returns PWM duty (-1..1) for wheel i
float updateWheel(int i, float goal, float dt, bool stopping) {
  WheelControl &w = wheel_ctrl[i];
  w.target = rampTowards(w.target, goal, MAX_WHEEL_ACCEL * dt);

#if USE_ENCODERS
  int64_t count = encoderCount(i);
  int64_t delta = count - w.last_count;
  w.last_count = count;
  float raw = ((float)delta / ENCODER_TICKS_PER_REV) * TWO_PI / dt;
  w.measured = 0.6f * w.measured + 0.4f * raw;  // light low-pass filter

  if (stopping && fabsf(w.target) < 0.05f) {
    w.integral = 0.0f;
    return 0.0f;
  }
  float error = w.target - w.measured;
  w.integral = constrain(w.integral + PID_KI * error * dt, -PID_I_LIMIT, PID_I_LIMIT);
  return w.target / MAX_WHEEL_SPEED + PID_KP * error + w.integral;
#else
  (void)stopping;
  w.measured = w.target;  // no feedback: report commanded speed for odometry
  return w.target / MAX_WHEEL_SPEED;
#endif
}

void controlStep(float dt) {
  bool cmd_fresh = (millis() - last_cmd_ms) < CMD_VEL_TIMEOUT_MS;
  float v = cmd_fresh ? cmd_linear : 0.0f;
  float w = cmd_fresh ? cmd_angular : 0.0f;

  // 4-wheel differential drive (skid steer) inverse kinematics
  //   v = linear speed (m/s), w = yaw rate (rad/s), W = TRACK_WIDTH, r = WHEEL_RADIUS
  //   Front and rear wheels on the same side are mechanically in line,
  //   so they must turn at the same speed:
  //     w_FL = w_RL = (v - w * W / 2) / r
  //     w_FR = w_RR = (v + w * W / 2) / r
  float goal[4];
  goal[WHEEL_FL] = (v - w * TRACK_WIDTH / 2.0f) / WHEEL_RADIUS;
  goal[WHEEL_RL] = goal[WHEEL_FL];
  goal[WHEEL_FR] = (v + w * TRACK_WIDTH / 2.0f) / WHEEL_RADIUS;
  goal[WHEEL_RR] = goal[WHEEL_FR];

  // Scale all wheels together so the turning ratio is kept when saturated
  float biggest = 0.0f;
  for (float g : goal) biggest = max(biggest, fabsf(g));
  if (biggest > MAX_WHEEL_SPEED) {
    for (float &g : goal) g *= MAX_WHEEL_SPEED / biggest;
  }

  bool stopping = !cmd_fresh || (v == 0.0f && w == 0.0f);
  for (int i = 0; i < 4; i++) {
    motorWrite(i, updateWheel(i, goal[i], dt, stopping));
  }
}

void resetControl() {
  for (int i = 0; i < 4; i++) {
    wheel_ctrl[i] = WheelControl();
    wheel_ctrl[i].last_count = encoderCount(i);
  }
  cmd_linear = 0.0f;
  cmd_angular = 0.0f;
}

// ---------------------------------------------------------------------------
// micro-ROS
// ---------------------------------------------------------------------------
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t control_timer;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t wheel_vel_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Vector3 wheel_vel_msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

void cmdVelCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_linear = msg->linear.x;
  cmd_angular = msg->angular.z;
  last_cmd_ms = millis();
}

void controlTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  static uint32_t last_us = micros();
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > 0.2f) dt = CONTROL_PERIOD_MS / 1000.0f;

  controlStep(dt);

  static int tick = 0;
  if (++tick >= PUBLISH_EVERY_N) {
    tick = 0;
    wheel_vel_msg.x = (wheel_ctrl[WHEEL_FL].measured + wheel_ctrl[WHEEL_RL].measured) / 2.0f;
    wheel_vel_msg.y = (wheel_ctrl[WHEEL_FR].measured + wheel_ctrl[WHEEL_RR].measured) / 2.0f;
    wheel_vel_msg.z = 0.0;
    RCSOFTCHECK(rcl_publish(&wheel_vel_pub, &wheel_vel_msg, NULL));
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "golf_bot_esp32", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_best_effort(
    &wheel_vel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "wheel_vel"));

  RCCHECK(rclc_timer_init_default2(
    &control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), controlTimerCallback, true));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  return true;
}

void destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&wheel_vel_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&cmd_vel_sub, &node));
  RCSOFTCHECK(rcl_timer_fini(&control_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  driveHwSetup();
  stopAllMotors();

  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial);
  delay(500);
}

void loop() {
  switch (agent_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)));
      break;
    case AGENT_AVAILABLE:
      resetControl();
      agent_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state == WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (agent_state == AGENT_CONNECTED) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      agent_state = WAITING_AGENT;
      break;
  }

  if (agent_state != AGENT_CONNECTED) {
    stopAllMotors();
  }
}
