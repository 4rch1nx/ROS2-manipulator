#include <micro_ros_arduino.h>
#include <SCServo.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>

// === WiFi Configuration ===
const char* ssid = "rosnet";
const char* password = "12345678";

// === ROS2 Agent Configuration ===
// IP of your computer running micro_ros_agent
const char* ros_agent_ip = "ros-FreeBook.local";  // Change to your PC's IP
const int ros_agent_port = 8888;

// === Hardware Configuration ===
SMS_STS st;
#define S_RXD 12  // GPIO12 - RX for servo
#define S_TXD 14  // GPIO13 - TX for servo
#define LED_PIN 2 // Built-in LED

// === ROS2 Variables ===
rcl_node_t node;
rcl_subscription_t servo_sub;
rcl_subscription_t led_sub;
std_msgs__msg__Int32MultiArray servo_cmd;
std_msgs__msg__Bool led_cmd;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// === Servo Parameters ===
const int SERVO_ID = 5;
bool wifi_connected = false;

// === Function Declarations ===
void connectToWiFi();
void setupROS();
void servo_callback(const void *msg_in);
void led_callback(const void *msg_in);
void blinkLED(int times, int delay_ms);

// === Callbacks ===
void servo_callback(const void *msg_in) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;
  
  if(msg->data.size >= 3) {
    int servo_id = msg->data.data[0];
    int position = msg->data.data[1];
    int speed = msg->data.data[2];
    
    // Clamp values
    servo_id = constrain(servo_id, 1, 253);
    position = constrain(position, 0, 4095);
    speed = constrain(speed, 0, 3400);
    
    // Send command to servo
    st.WritePosEx(servo_id, position, speed, 50);
    
    // Visual feedback
    blinkLED(1, 100);
    
    // Optional: Serial debug
    Serial.printf("Servo %d -> Pos: %d, Speed: %d\n", servo_id, position, speed);
  }
}

void led_callback(const void *msg_in) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

void connectToWiFi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    blinkLED(1, 100);
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Success pattern
    blinkLED(3, 200);
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed!");
    // Error pattern
    blinkLED(5, 100);
  }
}

void setupROS() {
  // Set micro-ROS transport to WiFi
  IPAddress agent_ip;
  agent_ip.fromString(ros_agent_ip);
  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, ros_agent_port);
  
  delay(2000);  // Wait for transport to initialize
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_wifi_node", "", &support));
  
  // Initialize servo command subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_control"
  ));
  
  // Initialize LED subscriber
  RCCHECK(rclc_subscription_init_default(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control"
  ));
  
  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_cmd, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_cmd, &led_callback, ON_NEW_DATA));
  
  // Initialize servo command message
  servo_cmd.data.capacity = 3;
  servo_cmd.data.size = 3;
  servo_cmd.data.data = (int32_t*)malloc(3 * sizeof(int32_t));
  
  Serial.println("ROS2 setup complete!");
  blinkLED(2, 200);
}

void blinkLED(int times, int delay_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delay_ms);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Setup hardware
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize servo communication
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  
  delay(1000);
  
  // Connect to WiFi
  connectToWiFi();
  
  if (wifi_connected) {
    // Setup ROS2
    setupROS();
    
    // Move servo to center position
    st.WritePosEx(SERVO_ID, 2048, 500, 50);
    delay(1000);
    
    Serial.println("ESP32 ready! Waiting for commands...");
  }
}

void loop() {
  if (wifi_connected) {
    // Process ROS2 messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  } else {
    // Blink slowly if not connected
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    
    // Try to reconnect every 30 seconds
    static unsigned long last_reconnect = 0;
    if (millis() - last_reconnect > 30000) {
      connectToWiFi();
      if (wifi_connected) {
        setupROS();
      }
      last_reconnect = millis();
    }
  }
}

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("ROS Error: %d\n", temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
