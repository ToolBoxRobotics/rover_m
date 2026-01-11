/*
 * OPPORTUNITY ROVER BASE CONTROLLER - ARDUINO MEGA #1
 * Protocol: ROS Noetic (rosserial)
 * Kinematics: 6WD + 4WS (Double Ackermann / Spot Turn mixing)
 */

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// --- ROBOT DIMENSIONS (METERS) ---
// YOU MUST MEASURE YOUR ROBOT!
const double TRACK_WIDTH = 0.40;  // Distance between Left and Right wheels
const double WHEELBASE   = 0.50;  // Distance between Front and Rear axles
const double WHEEL_RAD   = 0.06;  // Radius of the wheel

// --- PIN DEFINITIONS ---
// Motor Drivers (DRI0002) - Example mapping
// Driver 1 (Front)
#define M_FL_PWM 4
#define M_FL_DIR 5
#define M_FR_PWM 6
#define M_FR_DIR 7

// Driver 2 (Middle - Used for Odometry)
#define M_ML_PWM 8
#define M_ML_DIR 9
#define M_MR_PWM 10
#define M_MR_DIR 11

// Driver 3 (Rear)
#define M_RL_PWM 2
#define M_RL_DIR 3
#define M_RR_PWM 12
#define M_RR_DIR 13

// Encoders (Middle Wheels Only for Stability)
// Use Interrupt Pins: 18, 19, 20, 21 on Mega
#define ENC_ML_A 18
#define ENC_ML_B 19
#define ENC_MR_A 20
#define ENC_MR_B 21

// I2C Multiplexer
#define TCA_ADDR 0x70

// --- OBJECTS ---
ros::NodeHandle nh;
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(); // 0x40 default
MPU6050 accelgyro;

// --- ROS MESSAGES ---
nav_msgs::Odometry odom_msg;
sensor_msgs::BatteryState bat_msg;
sensor_msgs::Imu imu_msg;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// --- VARIABLES ---
volatile long pos_left = 0;
volatile long pos_right = 0;
double x = 0.0;
double y = 0.0;
double theta = 0.0;
unsigned long last_time = 0;

// Servo Calibration (Pulse lengths for 0 deg and 180 deg)
// Tweak these to center your wheels!
#define SERVOMIN  150 
#define SERVOMAX  600 

// --- HELPER FUNCTIONS ---

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Convert Angle (-90 to 90) to PWM Pulse
int angleToPulse(float angle) {
  // Map angle (-90 to 90) to (0 to 180) -> Pulse
  float mapped = map(angle, -90, 90, 0, 180);
  return map(mapped, 0, 180, SERVOMIN, SERVOMAX);
}

// Set Motor Speed (-255 to 255)
void setMotor(int pwm_pin, int dir_pin, int speed) {
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, speed);
  } else {
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, abs(speed));
  }
}

// --- CALLBACK: VELOCITY COMMAND ---
void cmdVelCb(const geometry_msgs::Twist& msg) {
  float v = msg.linear.x;   // Forward speed (m/s)
  float w = msg.angular.z;  // Turning speed (rad/s)
  
  // 1. SPOT TURN MODE (Spin in place)
  if (abs(v) < 0.01 && abs(w) > 0.01) {
    // Set servos to 45 degrees to form a circle
    // FL: -45, FR: 45, RL: 45, RR: -45 (Signs depend on servo mounting)
    servos.setPWM(0, 0, angleToPulse(-45)); // FL
    servos.setPWM(1, 0, angleToPulse(45));  // FR
    servos.setPWM(2, 0, angleToPulse(45));  // RL
    servos.setPWM(3, 0, angleToPulse(-45)); // RR
    
    // Motors oppose each other
    int pwm = constrain(w * 100, -255, 255); // Simple scaling
    setMotor(M_FL_PWM, M_FL_DIR, -pwm);
    setMotor(M_FR_PWM, M_FR_DIR, pwm);
    setMotor(M_ML_PWM, M_ML_DIR, -pwm);
    setMotor(M_MR_PWM, M_MR_DIR, pwm);
    setMotor(M_RL_PWM, M_RL_DIR, -pwm);
    setMotor(M_RR_PWM, M_RR_DIR, pwm);
  }
  
  // 2. ACKERMANN / STRAIGHT MODE
  else {
    float steer_angle = 0;
    float left_speed = v;
    float right_speed = v;

    if (abs(w) > 0.01) {
      // Calculate Turning Radius
      float R = v / w;
      
      // Calculate Inner/Outer angles (Simplified Bicycle Model for simplicity)
      steer_angle = atan(WHEELBASE / R) * (180.0 / PI); 
      
      // Differential Speed for Wheels
      left_speed  = w * (R - TRACK_WIDTH / 2.0);
      right_speed = w * (R + TRACK_WIDTH / 2.0);
    }
    
    // Clamp Servo Angles
    steer_angle = constrain(steer_angle, -45, 45);

    // Update Servos (Front and Rear steer opposite for tighter radius)
    servos.setPWM(0, 0, angleToPulse(steer_angle));  // FL
    servos.setPWM(1, 0, angleToPulse(steer_angle));  // FR
    servos.setPWM(2, 0, angleToPulse(-steer_angle)); // RL (Opposite)
    servos.setPWM(3, 0, angleToPulse(-steer_angle)); // RR (Opposite)

    // Map m/s to PWM (Simple linear mapping - calibrate this!)
    // Assuming 1.0 m/s = 255 PWM
    int pwm_l = constrain(left_speed * 255, -255, 255);
    int pwm_r = constrain(right_speed * 255, -255, 255);

    setMotor(M_FL_PWM, M_FL_DIR, pwm_l);
    setMotor(M_ML_PWM, M_ML_DIR, pwm_l);
    setMotor(M_RL_PWM, M_RL_DIR, pwm_l);

    setMotor(M_FR_PWM, M_FR_DIR, pwm_r);
    setMotor(M_MR_PWM, M_MR_DIR, pwm_r);
    setMotor(M_RR_PWM, M_RR_DIR, pwm_r);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCb);
ros::Publisher pub_odom("odom", &odom_msg);
ros::Publisher pub_imu("imu/data", &imu_msg);

// --- ENCODER INTERRUPTS ---
void readEncML() {
  if (digitalRead(ENC_ML_B) == HIGH) pos_left++;
  else pos_left--;
}
void readEncMR() {
  if (digitalRead(ENC_MR_B) == HIGH) pos_right++;
  else pos_right--;
}

// --- SETUP ---
void setup() {
  // 1. Init Serial/I2C
  Wire.begin();
  servos.begin();
  servos.setPWMFreq(60); // Analog servos run at ~60 Hz

  // 2. Init Sensors
  tcaSelect(0); // Assuming IMU is on Port 0
  accelgyro.initialize();

  // 3. Init Encoders
  pinMode(ENC_ML_A, INPUT_PULLUP);
  pinMode(ENC_ML_B, INPUT_PULLUP);
  pinMode(ENC_MR_A, INPUT_PULLUP);
  pinMode(ENC_MR_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_ML_A), readEncML, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_MR_A), readEncMR, RISING);

  // 4. Init Motors (Pins)
  // ... (Set all motor pins to OUTPUT) ...
  pinMode(M_FL_PWM, OUTPUT); pinMode(M_FL_DIR, OUTPUT);
  // ... (Repeat for all 6 motors)

  // 5. Init ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_odom);
  nh.advertise(pub_imu);
  broadcaster.init(nh);
}

// --- LOOP ---
void loop() {
  unsigned long current_time = millis();
  
  if (current_time - last_time >= 50) { // 20Hz Update Rate
    double dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // --- ODOMETRY CALCULATION ---
    // 1. Get ticks since last loop
    static long last_pos_left = 0;
    static long last_pos_right = 0;
    
    long d_left_ticks = pos_left - last_pos_left;
    long d_right_ticks = pos_right - last_pos_right;
    
    last_pos_left = pos_left;
    last_pos_right = pos_right;

    // 2. Convert to Distance
    // TICKS_PER_METER needs calibration!
    double TICKS_PER_METER = 3500.0; 
    double d_left = d_left_ticks / TICKS_PER_METER;
    double d_right = d_right_ticks / TICKS_PER_METER;

    // 3. Differential Drive Math
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / TRACK_WIDTH;

    // 4. Update Global Position
    double dx = d_center * cos(theta);
    double dy = d_center * sin(theta);

    x += dx;
    y += dy;
    theta += d_theta;

    // --- IMU FUSION (Simple Gyro Check) ---
    // Ideally, replace 'theta' above with Gyro Yaw if encoders slip
    // tcaSelect(0);
    // int16_t gx, gy, gz;
    // accelgyro.getRotation(&gx, &gy, &gz);
    // double gyro_z = gz / 131.0; // LSB sensitivity

    // --- PUBLISH ODOMETRY ---
    // 1. TF Transform (Base_Link to Odom)
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.header.stamp = nh.now();
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    broadcaster.sendTransform(t);

    // 2. Odom Message
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation = t.transform.rotation;
    
    // Twist (Velocity)
    odom_msg.twist.twist.linear.x = d_center / dt;
    odom_msg.twist.twist.angular.z = d_theta / dt;
    
    pub_odom.publish(&odom_msg);
  }

  nh.spinOnce();
}
