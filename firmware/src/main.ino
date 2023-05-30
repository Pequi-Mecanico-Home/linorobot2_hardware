// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
// #include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>


// #include <tf2/LinearMath/Quaternion.h>
// #include <Quaternion.h>


// #include "config.h"
// #include "motor.h"
// #include "kinematics.h"
// #include "pid.h"
// #include "odometry.h"
// #include "imu.h"
// #define ENCODER_USE_INTERRUPTS
// #define ENCODER_OPTIMIZE_INTERRUPTS
// #include "encoder.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
// sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;


// Luis Code
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval


uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
unsigned long iTimeFeedback = 0;
unsigned long iPeriodFeedback = 0;

float theta = 0;
int telem[2];


typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;


enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
// Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
// Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
// Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

// Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
// Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
// Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
// Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Kinematics kinematics(
//     Kinematics::LINO_BASE, 
//     MOTOR_MAX_RPM, 
//     MAX_RPM_RATIO, 
//     MOTOR_OPERATING_VOLTAGE, 
//     MOTOR_POWER_MAX_VOLTAGE, 
//     WHEEL_DIAMETER, 
//     LR_WHEELS_DISTANCE
// );

// Odometry odometry;
// IMU imu;

void setup() 
{
    // pinMode(LED_PIN, OUTPUT);

    // bool imu_ok = imu.init();
    // if(!imu_ok)
    // {
    //     while(1)
    //     {
    //         flashLED(3);
    //     }
    // }
    Serial2.begin(HOVER_SERIAL_BAUD);

    set_microros_serial_transports(Serial2);
}

unsigned long iTimeSend = 0;

void loop() {
    unsigned long timeNow = millis();
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                Receive();
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    iTimeSend = timeNow + TIME_SEND;
}


void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       Odom(); 
       moveBase();
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    prev_cmd_time = millis();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "miss_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    // create IMU publisher
    // RCCHECK(rclc_publisher_init_default( 
    //     &imu_publisher, 
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "imu/data"
    // ));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_BUILTIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    // rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_BUILTIN, HIGH);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    // twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    // motor1_controller.brake();
    // motor2_controller.brake();
    // motor3_controller.brake();
    // motor4_controller.brake();
}


// auto createQuaternionMsgFromYaw(double yaw)
// {
//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   return tf2::toMsg(q);
// }

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command));
}

void Odom() {
  odom_msg.twist.twist.linear.x = PI * 0.125 * float(Feedback.speedL_meas - Feedback.speedR_meas) / 60.0; // velocidade linear em m/s
  odom_msg.twist.twist.angular.z = 2.0 * PI * 0.125 * float(Feedback.speedR_meas + Feedback.speedL_meas) / (0.45 * 60.0); // velocidade angular em rad/s
  theta += odom_msg.twist.twist.angular.z * float(iPeriodFeedback) / 1000.0; // angulo em rad
  // odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
  odom_msg.pose.pose.position.x += odom_msg.twist.twist.linear.x * cos(theta) * float(iPeriodFeedback) / 1000.0; // distancia em m
  odom_msg.pose.pose.position.y += odom_msg.twist.twist.linear.x * sin(theta) * float(iPeriodFeedback) / 1000.0; // distancia em m
  telem[0] = Feedback.batVoltage;
  telem[1] = Feedback.boardTemp;
}

void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available()) {
    incomingByte    = Serial2.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  //  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p       = (byte *)&NewFeedback;
    *p++    = incomingBytePrev;
    *p++    = incomingByte;
    idx     = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++    = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      if (iTimeFeedback == 0) {
        iTimeFeedback = millis();
        iPeriodFeedback = millis();
      }
      else {
        iPeriodFeedback = millis() - iTimeFeedback;
        iTimeFeedback = millis();
      }

      // Print data to built-in Serial
      //                  Serial.print("1: ");   Serial.print(Feedback.cmd1);
      //                  Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      //                  Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      //                  Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      //                  Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      //                  Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      //                  Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      //      Serial.println("Non - valid data skipped");
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}



void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        // twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_BUILTIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    // Kinematics::rpm req_rpm = kinematics.getRPM(
    //     twist_msg.linear.x, 
    //     twist_msg.linear.y, 
    //     twist_msg.angular.z
    // );

    // get the current speed of each motor
    // float current_rpm1 = motor1_encoder.getRPM();
    // float current_rpm2 = motor2_encoder.getRPM();
    // float current_rpm3 = motor3_encoder.getRPM();
    // float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    // motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    // motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    // motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    // motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Send(77.0 * twist_msg.linear.x, 34.0 * twist_msg.angular.z);

    // Kinematics::velocities current_vel = kinematics.getVelocities(
    //     current_rpm1, 
    //     current_rpm2, 
    //     current_rpm3, 
    //     current_rpm4
    // );

    // unsigned long now = millis();
    // float vel_dt = (now - prev_odom_update) / 1000.0;
    // prev_odom_update = now;
    // odometry.update(
    //     vel_dt, 
    //     current_vel.linear_x, 
    //     current_vel.linear_y, 
    //     current_vel.angular_z
    // );
    
}

void publishData()
{
    // odom_msg = odometry.getData();
    // imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // imu_msg.header.stamp.sec = time_stamp.tv_sec;
    // imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }
    delay(1000);
}