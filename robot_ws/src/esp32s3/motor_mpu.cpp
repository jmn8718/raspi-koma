#include <micro_ros_arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

// --- USER PINS ---
#define SDA_PIN 8
#define SCL_PIN 9
#define ENA 4
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 15
#define ENB 16

Adafruit_MPU6050 mpu;
rcl_subscription_t cmd_sub;
rcl_publisher_t imu_pub;
geometry_msgs__msg__Twist msg_cmd;
sensor_msgs__msg__Imu msg_imu;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Shared variables (protected by volatile for cross-core safety)
volatile float motor_linear = 0;
volatile float motor_angular = 0;

// Task Handles
TaskHandle_t ControlTask;

// --- MOTOR CONTROL LOGIC ---
void set_motors(float left, float right) {
  analogWrite(ENA, abs(left) * 255);
  digitalWrite(IN1, left > 0); digitalWrite(IN2, left < 0);
  analogWrite(ENB, abs(right) * 255);
  digitalWrite(IN3, right > 0); digitalWrite(IN4, right < 0);
}

// --- CORE 1: REAL-TIME TASK (Motors & IMU) ---
void ControlLoop(void * pvParameters) {
  for(;;) {
    // 1. Process Motors based on shared values
    float left = motor_linear - motor_angular;
    float right = motor_linear + motor_angular;
    set_motors(left, right);

    // 2. Read IMU Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    msg_imu.linear_acceleration.x = a.acceleration.x;
    msg_imu.linear_acceleration.y = a.acceleration.y;
    msg_imu.linear_acceleration.z = a.acceleration.z;
    msg_imu.angular_velocity.x = g.gyro.x;
    msg_imu.angular_velocity.y = g.gyro.y;
    msg_imu.angular_velocity.z = g.gyro.z;
    
    // Small delay to prevent task hogging (100Hz loop)
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// --- ROS CALLBACK (CORE 0) ---
void cmd_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
  motor_linear = msg->linear.x;
  motor_angular = msg->angular.z;
}

void setup() {
  set_microros_transports();
  
  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) { while(1) delay(10); }

  // Initialize Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Micro-ROS Setup
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_s3_node", "", &support);

  rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &msg_cmd, &cmd_callback, ON_NEW_DATA);

  // START REAL-TIME TASK ON CORE 1
  xTaskCreatePinnedToCore(
    ControlLoop,    /* Task function */
    "ControlTask",  /* Name */
    10000,          /* Stack size */
    NULL,           /* Parameter */
    1,              /* Priority */
    &ControlTask,   /* Handle */
    1               /* Core 1 */
  );
}

void loop() {
  // CORE 0 handles the high-bandwidth ROS 2 communication
  rcl_publish(&imu_pub, &msg_imu, NULL);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}