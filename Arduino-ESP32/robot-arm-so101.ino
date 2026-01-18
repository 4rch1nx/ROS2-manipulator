#include <micro_ros_arduino.h>
#include <SCServo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>

SMS_STS st;

// Servo UART pins
#define S_RXD 12  // GPIO12 - RX
#define S_TXD 14  // GPIO13 - TX

// Built-in LED
#define LED_PIN 2

// ROS2 node and topics
rcl_node_t node;
rcl_subscription_t servo_sub;
rcl_subscription_t led_sub;
std_msgs__msg__Int32MultiArray servo_cmd;
std_msgs__msg__Bool led_cmd;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Servo parameters
const int SERVO_MIN_POS = 158;
const int SERVO_MAX_POS = 3957;
const int SERVO_ID = 5;  // Default servo ID

void servo_callback(const void *msg_in) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;

  if (msg->data.size >= 3) {
    int servo_id = msg->data.data[0];
    int position = msg->data.data[1];
    int speed = msg->data.data[2];

    // Clamp values
    servo_id = constrain(servo_id, 1, 253);
    position = constrain(position, SERVO_MIN_POS, SERVO_MAX_POS);
    speed = constrain(speed, 0, 3400);

    // Send command to servo
    st.WritePosEx(servo_id, position, speed, 50);

    // Blink LED to indicate command received
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
}

void led_callback(const void *msg_in) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

void setup() {
  // Initialize micro-ROS
  set_microros_transports();

  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize servo communication
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  delay(2000);  // Wait for initialization

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_servo_node", "", &support);

  // Initialize servo command subscriber
  rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_control");

  // Initialize LED subscriber
  rclc_subscription_init_default(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_cmd, &servo_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub, &led_cmd, &led_callback, ON_NEW_DATA);

  // Initialize servo command message
  servo_cmd.data.capacity = 3;
  servo_cmd.data.size = 3;
  servo_cmd.data.data = (int32_t *)malloc(3 * sizeof(int32_t));

  // Set initial servo position (middle)
  st.WritePosEx(SERVO_ID, 2027, 500, 50);

  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Process ROS2 messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Add any periodic tasks here
}