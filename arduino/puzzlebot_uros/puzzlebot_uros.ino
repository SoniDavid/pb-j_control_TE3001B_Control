/*
 * PuzzleBot Micro-ROS Node — Dual Motor / Dual Encoder
 * ═══════════════════════════════════════════════════════
 * ESP32 firmware that bridges hardware ↔ ROS 2 via Micro-ROS (USB serial).
 *
 * SUBSCRIPTIONS
 *   /motor_left/cmd_pwm   std_msgs/Int16   Left wheel PWM  (−255 … +255)
 *   /motor_right/cmd_pwm  std_msgs/Int16   Right wheel PWM (−255 … +255)
 *
 * PUBLICATIONS (50 ms / 20 Hz)
 *   /motor_left/rpm       std_msgs/Float32  Left  wheel RPM (signed)
 *   /motor_right/rpm      std_msgs/Float32  Right wheel RPM (signed)
 *   /motor_left/encoder   std_msgs/Int32    Left  cumulative encoder count
 *   /motor_right/encoder  std_msgs/Int32    Right cumulative encoder count
 *
 * PIN MAP (ESP32 DevKit / Hackerboard)
 * ─────────────────────────────────────
 *  Left motor  │ IN1=26  IN2=25  ENA/PWM=27  EncA=18  EncB=19
 *  Right motor │ IN3=33  IN4=32  ENB/PWM=14  EncA=23  EncB=22
 *  Status LED  │ 2
 *
 * To start the agent on the PC side:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
// Left motor (L298N channel A)
#define L_IN1       26
#define L_IN2       25
#define L_ENA       27      // PWM pin
#define L_ENC_A     18      // Interrupt – rising edge
#define L_ENC_B     19      // Direction

// Right motor (L298N channel B)
#define R_IN3       13
#define R_IN4       14
#define R_ENB       12      // PWM pin
#define R_ENC_A     15      // Interrupt – risinSg edge
#define R_ENC_B     4      // Direction

#define LED_PIN     2

// ═══════════════════════════════════════════════════════════════
//  PWM / ENCODER CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define PWM_FREQ        1000
#define PWM_RESOLUTION  8           // 8-bit → 0..255
#define PWM_CH_LEFT     0
#define PWM_CH_RIGHT    1

#define PULSES_PER_REV  495.0f      // matches original single-motor node
#define SAMPLE_TIME_MS  50          // 20 Hz control loop
#define CMD_TIMEOUT_MS  500         // stop if silent for 500 ms

// EMA low-pass filter coefficient (0 < α ≤ 1)
#define RPM_ALPHA       0.3f

// ═══════════════════════════════════════════════════════════════
//  MICRO-ROS ENTITIES
// ═══════════════════════════════════════════════════════════════
rclc_support_t   support;
rclc_executor_t  executor;
rcl_allocator_t  allocator;
rcl_node_t       node;

// Subscriptions
rcl_subscription_t sub_left_pwm;
rcl_subscription_t sub_right_pwm;

// Publishers
rcl_publisher_t pub_left_rpm;
rcl_publisher_t pub_right_rpm;
rcl_publisher_t pub_left_enc;
rcl_publisher_t pub_right_enc;

// Timer
rcl_timer_t control_timer;

// Messages
std_msgs__msg__Int16   msg_left_pwm;
std_msgs__msg__Int16   msg_right_pwm;
std_msgs__msg__Float32 msg_left_rpm;
std_msgs__msg__Float32 msg_right_rpm;
std_msgs__msg__Int32   msg_left_enc;
std_msgs__msg__Int32   msg_right_enc;

// ═══════════════════════════════════════════════════════════════
//  ERROR HANDLING
// ═══════════════════════════════════════════════════════════════
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK){ return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

#define EXECUTE_EVERY_N_MS(MS, X) do {            \
  static volatile int64_t _t = -1;                \
  if (_t == -1) { _t = uxr_millis(); }            \
  if (uxr_millis() - _t > MS) { X; _t = uxr_millis(); } \
} while(0)

// ═══════════════════════════════════════════════════════════════
//  MOTOR STATE
// ═══════════════════════════════════════════════════════════════
volatile long     encoderLeft  = 0;
volatile long     encoderRight = 0;
volatile int16_t  cmdLeft      = 0;
volatile int16_t  cmdRight     = 0;
volatile uint32_t lastCmdLeftMs  = 0;
volatile uint32_t lastCmdRightMs = 0;

long  prevEncLeft  = 0,  prevEncRight  = 0;
float filtRpmLeft  = 0.0f, filtRpmRight = 0.0f;
uint32_t prevMs    = 0;

// ═══════════════════════════════════════════════════════════════
//  CONNECTION STATE MACHINE
// ═══════════════════════════════════════════════════════════════
enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state = WAITING_AGENT;

// ─── Forward declarations ────────────────────────────────────
bool create_entities();
void destroy_entities();
void apply_motor(uint8_t in1, uint8_t in2, uint8_t ch, int16_t cmd);

// ═══════════════════════════════════════════════════════════════
//  ENCODER ISRs
// ═══════════════════════════════════════════════════════════════
void IRAM_ATTR isrLeftA() {
  encoderLeft  += (digitalRead(L_ENC_B)) ? 1 : -1;
}
void IRAM_ATTR isrRightA() {
  encoderRight += (digitalRead(R_ENC_B)) ? 1 : -1;
}

// ═══════════════════════════════════════════════════════════════
//  SUBSCRIPTION CALLBACKS
// ═══════════════════════════════════════════════════════════════
void cb_left_pwm(const void *msgin) {
  const std_msgs__msg__Int16 *m = (const std_msgs__msg__Int16 *)msgin;
  cmdLeft         = (int16_t)constrain((int32_t)m->data, -255, 255);
  lastCmdLeftMs   = millis();
}

void cb_right_pwm(const void *msgin) {
  const std_msgs__msg__Int16 *m = (const std_msgs__msg__Int16 *)msgin;
  cmdRight        = (int16_t)constrain((int32_t)m->data, -255, 255);
  lastCmdRightMs  = millis();
}

// ═══════════════════════════════════════════════════════════════
//  CONTROL TIMER (20 Hz)
// ═══════════════════════════════════════════════════════════════
void control_timer_callback(rcl_timer_t *timer, int64_t /*last_call*/) {
  if (!timer) return;

  uint32_t now = millis();
  float dt = (now - prevMs) * 1e-3f;
  if (dt <= 0.0f) return;

  // Command-timeout safety stop
  if (lastCmdLeftMs  > 0 && (now - lastCmdLeftMs)  > CMD_TIMEOUT_MS) cmdLeft  = 0;
  if (lastCmdRightMs > 0 && (now - lastCmdRightMs) > CMD_TIMEOUT_MS) cmdRight = 0;

  // Read encoders atomically
  long encL, encR;
  noInterrupts();
  encL = encoderLeft;
  encR = encoderRight;
  interrupts();

  // RPM calculation
  float rawL = ((encL - prevEncLeft)  * 60.0f) / (PULSES_PER_REV * dt);
  float rawR = ((encR - prevEncRight) * 60.0f) / (PULSES_PER_REV * dt);

  // EMA filter (snap to 0 when stopped)
  if (cmdLeft  == 0 && rawL == 0.0f) filtRpmLeft  = 0.0f;
  else                                filtRpmLeft  = RPM_ALPHA * rawL + (1.0f - RPM_ALPHA) * filtRpmLeft;

  if (cmdRight == 0 && rawR == 0.0f) filtRpmRight = 0.0f;
  else                                filtRpmRight = RPM_ALPHA * rawR + (1.0f - RPM_ALPHA) * filtRpmRight;

  prevEncLeft  = encL;
  prevEncRight = encR;
  prevMs       = now;

  // Publish RPM
  msg_left_rpm.data  = filtRpmLeft;
  msg_right_rpm.data = filtRpmRight;
  RCSOFTCHECK(rcl_publish(&pub_left_rpm,  &msg_left_rpm,  NULL));
  RCSOFTCHECK(rcl_publish(&pub_right_rpm, &msg_right_rpm, NULL));

  // Publish encoder counts
  msg_left_enc.data  = (int32_t)encL;
  msg_right_enc.data = (int32_t)encR;
  RCSOFTCHECK(rcl_publish(&pub_left_enc,  &msg_left_enc,  NULL));
  RCSOFTCHECK(rcl_publish(&pub_right_enc, &msg_right_enc, NULL));

  // Heartbeat LED
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ═══════════════════════════════════════════════════════════════
//  HELPER: apply PWM command to one motor
//   in1/in2 : direction GPIO pair
//   ch      : LEDC channel
//   cmd     : −255…+255  (positive = forward, negative = reverse, 0 = brake)
// ═══════════════════════════════════════════════════════════════
void apply_motor(uint8_t in1, uint8_t in2, uint8_t ch, int16_t cmd) {
  uint8_t duty;
  if (cmd == 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);   // brake
    duty = 0;
  } else if (cmd > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);    // forward
    duty = (uint8_t)cmd;
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);   // reverse
    duty = (uint8_t)(-cmd);
  }
  ledcWrite(ch, duty);
}

// ═══════════════════════════════════════════════════════════════
//  ARDUINO SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== PuzzleBot uROS Node ===");

  // Micro-ROS transport (USB serial by default)
  set_microros_transports();

  // Motor direction GPIOs
  for (uint8_t p : {L_IN1, L_IN2, R_IN3, R_IN4, LED_PIN}) {
    pinMode(p, OUTPUT);
  }

  // Encoder GPIOs
  for (uint8_t p : {L_ENC_A, L_ENC_B, R_ENC_A, R_ENC_B}) {
    pinMode(p, INPUT_PULLUP);
  }

  // Safe initial state  (brake both motors)
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, HIGH);
  digitalWrite(LED_PIN, LOW);

  // LEDC PWM channels
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_ENA, PWM_CH_LEFT);
  ledcAttachPin(R_ENB, PWM_CH_RIGHT);
  ledcWrite(PWM_CH_LEFT,  0);
  ledcWrite(PWM_CH_RIGHT, 0);

  // Encoder interrupts (RISING only — ISR checks PhaseB for direction)
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrRightA, RISING);

  prevMs = millis();
  state  = WAITING_AGENT;
  Serial.println("Setup done. Waiting for ROS 2 agent...");
}

// ═══════════════════════════════════════════════════════════════
//  ARDUINO LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      if (state == AGENT_CONNECTED) Serial.println("Connected to ROS 2 agent!");
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        apply_motor(L_IN1, L_IN2, PWM_CH_LEFT,  cmdLeft);
        apply_motor(R_IN3, R_IN4, PWM_CH_RIGHT, cmdRight);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("Agent disconnected – stopping motors.");
      destroy_entities();
      cmdLeft = cmdRight = 0;
      apply_motor(L_IN1, L_IN2, PWM_CH_LEFT,  0);
      apply_motor(R_IN3, R_IN4, PWM_CH_RIGHT, 0);
      state = WAITING_AGENT;
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  CREATE ROS 2 ENTITIES
// ═══════════════════════════════════════════════════════════════
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "puzzlebot_node", "", &support));

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(
      &sub_left_pwm, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "motor_left/cmd_pwm"));

  RCCHECK(rclc_subscription_init_default(
      &sub_right_pwm, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "motor_right/cmd_pwm"));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &pub_left_rpm, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor_left/rpm"));

  RCCHECK(rclc_publisher_init_default(
      &pub_right_rpm, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor_right/rpm"));

  RCCHECK(rclc_publisher_init_default(
      &pub_left_enc, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "motor_left/encoder"));

  RCCHECK(rclc_publisher_init_default(
      &pub_right_enc, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "motor_right/encoder"));

  // Control timer  (50 ms = 20 Hz)
  RCCHECK(rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS(SAMPLE_TIME_MS),
      control_timer_callback));

  // Executor: 2 subscriptions + 1 timer = 3 handles
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_left_pwm,  &msg_left_pwm,  &cb_left_pwm,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_right_pwm, &msg_right_pwm, &cb_right_pwm, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  Serial.println("ROS 2 entities created.");
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  DESTROY ROS 2 ENTITIES
// ═══════════════════════════════════════════════════════════════
void destroy_entities() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

  rcl_subscription_fini(&sub_left_pwm,  &node);
  rcl_subscription_fini(&sub_right_pwm, &node);
  rcl_publisher_fini(&pub_left_rpm,   &node);
  rcl_publisher_fini(&pub_right_rpm,  &node);
  rcl_publisher_fini(&pub_left_enc,   &node);
  rcl_publisher_fini(&pub_right_enc,  &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
