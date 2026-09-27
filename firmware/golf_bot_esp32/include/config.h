// Golf ball collector robot: hardware configuration.
// Pin map follows "GOLF COLLECTING ROBOT" schematic V1.0, page 2 (Drive System).
#pragma once

// ---------------------------------------------------------------------------
// Robot geometry (must match golf_bot_bringup/config/golf_bot.yaml)
// ---------------------------------------------------------------------------
#define WHEEL_RADIUS        0.100f  // m  (100 mm)
#define TRACK_WIDTH         0.510f  // m  (510 mm, wheel center to wheel center; calibrate for skid steer)

// ---------------------------------------------------------------------------
// Motors: 2x Cytron MDD10A (sign-magnitude: PWM = speed, DIR = direction)
//   D1: M1 (PWM1/DIR1), M2 (PWM2/DIR2)
//   D2: M3 (PWM1/DIR1), M4 (PWM2/DIR2)
// Encoders: 36GP-555 magnetic encoder on every motor (phase A / phase B)
//
//                 PWM  DIR  ENC_A  ENC_B  invert_motor  invert_encoder
#define M1_CONFIG  16,  15,  13,    12,    true,         false
#define M2_CONFIG  18,  17,  14,    27,    false,         true
#define M3_CONFIG  21,  19,  26,    25,    true,         false
#define M4_CONFIG  4,   5,   33,    32,    false,         true    // moved from GPIO23/22 (no signal at D2 ch2)
//
// invert_motor:   flip if the wheel spins backward when commanded forward
// invert_encoder: flip if the count goes down while the wheel spins forward
// Check both with the motor_test program (see README) before driving.
// ---------------------------------------------------------------------------

// Which motor sits at which corner (0 = M1, 1 = M2, 2 = M3, 3 = M4)
#define WHEEL_FL  0   // front left  = M1
#define WHEEL_FR  1   // front right = M2
#define WHEEL_RL  2   // rear left   = M3
#define WHEEL_RR  3   // rear right  = M4

#define PWM_FREQ_HZ         20000   // MDD10A supports up to 20 kHz
#define PWM_RESOLUTION_BITS 10

// ---------------------------------------------------------------------------
// Encoder resolution (counted in full quadrature = 4 edges per pulse)
// TICKS_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO, counts per *wheel* revolution.
// Easiest: run motor_test, turn one wheel exactly 1 revolution by hand,
// and put the count it shows into ENCODER_TICKS_PER_REV directly.
// ---------------------------------------------------------------------------
#define USE_ENCODERS          1
#define ENCODER_PPR           7       // pulses per motor revolution (per phase)
#define GEAR_RATIO            19.2f   // gearbox ratio of your 36GP-555 version
#define ENCODER_TICKS_PER_REV (ENCODER_PPR * 4 * GEAR_RATIO)

// Open-loop feedforward: wheel speed (rad/s) at 100% PWM, measured with motor_test.
#define MAX_WHEEL_SPEED     20.0f   // rad/s
// Limit wheel acceleration so the skid-steer base does not jerk or tip.
#define MAX_WHEEL_ACCEL     40.0f   // rad/s^2

// Speed PID (per wheel), output is PWM fraction added on top of feedforward
#define PID_KP              0.03f
#define PID_KI              0.15f
#define PID_I_LIMIT         0.3f

// ---------------------------------------------------------------------------
// Safety / timing
// ---------------------------------------------------------------------------
#define CMD_VEL_TIMEOUT_MS   500    // stop wheels if no /cmd_vel for this long
#define CONTROL_PERIOD_MS    20     // 50 Hz control loop
#define PUBLISH_EVERY_N      2      // publish /wheel_vel at 25 Hz
#define STATUS_LED_PIN       2      // on-board LED: blink = waiting for agent, on = connected

#define SERIAL_BAUD          115200 // must match micro_ros_agent -b
