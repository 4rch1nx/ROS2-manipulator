#include <micro_ros_arduino.h>
#include <SCServo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <WiFi.h>  // Wi-Fi library for ESP32

SMS_STS st;

// ==================== Wi-Fi Configuration ====================
const char *WIFI_SSID = "rosnet";             // Change this
const char *WIFI_PASSWORD = "12345678";       // Change this
const char *AGENT_IP = "ros-FreeBook.local";  // Your PC's IP address
const int AGENT_PORT = 8888;                  // micro-ROS Agent port
// ===========================================================

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
const int SERVO_MIN_POS = 0;
const int SERVO_MAX_POS = 4096;
const int SERVO_ID = 5;  // Default servo ID

// Wi-Fi connection status
bool wifi_connected = false;

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

// Connect to Wi-Fi network
void connectToWiFi() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Agent IP: ");
    Serial.println(AGENT_IP);
    Serial.print("Agent Port: ");
    Serial.println(AGENT_PORT);
  } else {
    wifi_connected = false;
    Serial.println("\nWiFi connection failed!");
  }
}

void setup() {
  // ========== Step 1: Connect to Wi-Fi FIRST ==========
  connectToWiFi();

  // Wait a moment for network to stabilize
  delay(2000);

  // ========== Step 2: Setup LED ==========
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ========== Step 3: Initialize servo communication ==========
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  delay(1000);

  // ========== Step 4: Initialize micro-ROS with UDP transport ==========
  // This sets up micro-ROS to use Wi-Fi/UDP instead of Serial
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  // Alternative if above doesn't work (some micro-ROS versions):
  // set_microros_transports();  // Will use configured transport from menuconfig

  // ========== Step 5: Initialize ROS2 node ==========
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

  // Indicate setup complete
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  Serial.println("ESP32 micro-ROS setup complete!");
}

void loop() {
  // Reconnect Wi-Fi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    connectToWiFi();
    delay(2000);
  }

  // Process ROS2 messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Add any periodic tasks here
}
