/*
 * ROVER ARM CONTROLLER - ARDUINO MEGA #2
 * Protocol: ROS Noetic (rosserial)
 * Hardware: 5x NEMA17 (A4988), 1x Servo (Gripper), 5x Limit Switches
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <AccelStepper.h>
#include <Servo.h>

// --- PIN DEFINITIONS ---
// Adjust these to match your specific wiring
#define J1_STEP 2
#define J1_DIR  3
#define J1_LIM  22

#define J2_STEP 4
#define J2_DIR  5
#define J2_LIM  23

#define J3_STEP 6
#define J3_DIR  7
#define J3_LIM  24

#define J4_STEP 8
#define J4_DIR  9
#define J4_LIM  25

#define J5_STEP 10
#define J5_DIR  11
#define J5_LIM  26

#define GRIPPER_PIN 12
#define ENABLE_PIN  30 // Common enable pin for A4988s

// --- ROBOT CONSTANTS ---
const float STEPS_PER_REV = 200.0;
const float MICROSTEPS = 16.0; // Ensure A4988 jumpers match this
const float GEAR_RATIOS[] = {1.0, 1.0, 1.0, 1.0, 1.0}; // Update if using gearboxes
const float STEPS_PER_RAD = (STEPS_PER_REV * MICROSTEPS) / (2.0 * PI);

// --- OBJECTS ---
// Interface 1 = Driver (Step + Dir)
AccelStepper joint1(1, J1_STEP, J1_DIR);
AccelStepper joint2(1, J2_STEP, J2_DIR);
AccelStepper joint3(1, J3_STEP, J3_DIR);
AccelStepper joint4(1, J4_STEP, J4_DIR);
AccelStepper joint5(1, J5_STEP, J5_DIR);

Servo gripper;

ros::NodeHandle nh;

// --- STATE VARIABLES ---
bool is_calibrated = false;
long target_steps[5] = {0,0,0,0,0};

// --- ROS CALLBACKS ---

// Callback to receive joint angles (in Radians) from MoveIt
void armCb(const std_msgs::Float32MultiArray& msg) {
  if (!is_calibrated) return; // Ignore commands if not homed

  // Ensure we received data for all 5 joints + gripper
  if (msg.data_length >= 5) {
     long s1 = msg.data[0] * STEPS_PER_RAD * GEAR_RATIOS[0];
     long s2 = msg.data[1] * STEPS_PER_RAD * GEAR_RATIOS[1];
     long s3 = msg.data[2] * STEPS_PER_RAD * GEAR_RATIOS[2];
     long s4 = msg.data[3] * STEPS_PER_RAD * GEAR_RATIOS[3];
     long s5 = msg.data[4] * STEPS_PER_RAD * GEAR_RATIOS[4];
     
     joint1.moveTo(s1);
     joint2.moveTo(s2);
     joint3.moveTo(s3);
     joint4.moveTo(s4);
     joint5.moveTo(s5);
  }
  
  // Gripper Control (Index 5 in the array, range 0.0 closed to 1.0 open)
  if (msg.data_length >= 6) {
    int angle = map(msg.data[5] * 100, 0, 100, 0, 180); 
    gripper.write(angle);
  }
}

// Callback to trigger calibration manually via ROS
void calibrateCb(const std_msgs::Bool& msg) {
  if (msg.data) {
    calibrate_joints();
  }
}

// --- ROS SUBSCRIBERS ---
ros::Subscriber<std_msgs::Float32MultiArray> sub_arm("arm_joint_goals", &armCb);
ros::Subscriber<std_msgs::Bool> sub_cal("arm_calibrate", &calibrateCb);

// --- HELPER FUNCTIONS ---

void setup_stepper(AccelStepper &stepper, int max_speed, int accel) {
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(accel);
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // Invert Enable logic for A4988
  stepper.enableOutputs();
}

void calibrate_single_joint(AccelStepper &stepper, int limit_pin, int homing_dir) {
  // 1. Move fast towards switch
  stepper.setSpeed(homing_dir * 500); 
  while (digitalRead(limit_pin) == HIGH) { // Assuming pull-up, switch grounds pin
    stepper.runSpeed();
    nh.spinOnce(); // Keep ROS connection alive
  }
  
  // 2. Back off slightly
  stepper.setCurrentPosition(0);
  stepper.moveTo(-homing_dir * 200);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  // 3. Move slow to trigger point
  stepper.setSpeed(homing_dir * 100);
  while (digitalRead(limit_pin) == HIGH) {
    stepper.runSpeed();
    nh.spinOnce();
  }
  
  // 4. Zero the position
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
}

void calibrate_joints() {
  // Homing sequence - adjust directions (1 or -1) based on switch location
  calibrate_single_joint(joint1, J1_LIM, -1); 
  calibrate_single_joint(joint2, J2_LIM, -1);
  calibrate_single_joint(joint3, J3_LIM, -1);
  calibrate_single_joint(joint4, J4_LIM, -1);
  calibrate_single_joint(joint5, J5_LIM, -1);
  
  is_calibrated = true;
  nh.loginfo("Arm Calibration Complete.");
}

// --- MAIN SETUP ---

void setup() {
  // Define Limit Switches
  pinMode(J1_LIM, INPUT_PULLUP);
  pinMode(J2_LIM, INPUT_PULLUP);
  pinMode(J3_LIM, INPUT_PULLUP);
  pinMode(J4_LIM, INPUT_PULLUP);
  pinMode(J5_LIM, INPUT_PULLUP);
  
  // Setup Steppers (Max Speed, Acceleration)
  setup_stepper(joint1, 1000, 500);
  setup_stepper(joint2, 1000, 500);
  setup_stepper(joint3, 1000, 500);
  setup_stepper(joint4, 1000, 500);
  setup_stepper(joint5, 1000, 500);

  gripper.attach(GRIPPER_PIN);

  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub_arm);
  nh.subscribe(sub_cal);
}

// --- MAIN LOOP ---

void loop() {
  if (is_calibrated) {
    joint1.run();
    joint2.run();
    joint3.run();
    joint4.run();
    joint5.run();
  }
  nh.spinOnce();
}
