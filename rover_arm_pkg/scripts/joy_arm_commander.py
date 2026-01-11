#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from math import pi

# --- XBOX CONTROLLER MAP ---
# Axes
AXIS_L_STICK_X = 0
AXIS_L_STICK_Y = 1
AXIS_LT = 2
AXIS_R_STICK_X = 3
AXIS_R_STICK_Y = 4
AXIS_RT = 5
AXIS_DPAD_X = 6 # Left/Right
AXIS_DPAD_Y = 7 # Up/Down

# Buttons
BTN_A = 0 
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4 # Deadman for Driving
BTN_RB = 5 # Modifier for Z-axis
BTN_BACK = 6
BTN_START = 7

class JoyArmCommander:
    def __init__(self):
        rospy.init_node('joy_arm_commander')
        
        # 1. Init MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        
        # Optimization: continuous movement requires looser tolerances
        self.arm_group.set_goal_position_tolerance(0.005)
        self.arm_group.set_goal_orientation_tolerance(0.1)
        self.arm_group.set_planning_time(0.5)

        # 2. State
        self.processing = False
        self.step_size = 0.02 # 2cm per click/cycle
        
        # 3. Subscribe
        rospy.Subscriber("joy", Joy, self.joy_cb, queue_size=1)
        
        rospy.loginfo("--- ARM JOGGING READY ---")
        rospy.loginfo("D-Pad: Move X/Y | RB + D-Pad: Move Z")

    def joy_cb(self, msg):
        if self.processing:
            return

        # SAFETY: If LB is held, user is driving. Ignore arm inputs.
        if msg.buttons[BTN_LB] == 1:
            return

        # --- PART A: DISCRETE ACTIONS (Poses & Gripper) ---
        
        # Gripper Control
        if msg.buttons[BTN_Y] == 1: # OPEN
            self.move_gripper(1.0) # Open
            return
        elif msg.buttons[BTN_X] == 1: # CLOSE
            self.move_gripper(0.0) # Closed
            return
            
        # Poses
        if msg.buttons[BTN_A] == 1:
            self.move_to_pose("home")
            return
        elif msg.buttons[BTN_B] == 1:
            self.move_to_pose("stow")
            return

        # --- PART B: IK JOGGING (Cartesian) ---
        
        dpad_x = msg.axes[AXIS_DPAD_X] # Left/Right (1.0 left, -1.0 right)
        dpad_y = msg.axes[AXIS_DPAD_Y] # Up/Down (1.0 up, -1.0 down)
        rb_pressed = msg.buttons[BTN_RB]
        
        # Check if D-Pad is active
        if abs(dpad_x) > 0.1 or abs(dpad_y) > 0.1:
            self.processing = True
            
            # 1. Get current pose
            current_pose = self.arm_group.get_current_pose().pose
            target_pose = geometry_msgs.msg.Pose()
            target_pose.orientation = current_pose.orientation # Keep rotation same
            target_pose.position = current_pose.position # Start from current
            
            # 2. Calculate Delta
            # COORD FRAME: X=Forward, Y=Left, Z=Up (Relative to base_link)
            
            if rb_pressed:
                # Z-Axis Mode (RB + D-Pad Up/Down)
                target_pose.position.z += (dpad_y * self.step_size)
            else:
                # Planar Mode (X/Y)
                target_pose.position.x += (dpad_y * self.step_size) # Forward/Back
                target_pose.position.y += (dpad_x * self.step_size) # Left/Right
            
            # 3. Compute Cartesian Path
            waypoints = [target_pose]
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step (1cm interpolation)
                                   0.0)         # jump_threshold
                                   
            # 4. Execute if valid path found
            if fraction > 0.9: # Only move if 90% of path is possible
                self.arm_group.execute(plan, wait=True)
            else:
                rospy.logwarn("Cannot reach target (Singularity or Limit)")
                
            self.processing = False

    def move_gripper(self, val):
        self.processing = True
        self.hand_group.set_joint_value_target("gripper_joint", val)
        self.hand_group.go(wait=False)
        rospy.sleep(0.5)
        self.processing = False

    def move_to_pose(self, name):
        self.processing = True
        self.arm_group.set_named_target(name)
        self.arm_group.go(wait=True)
        self.processing = False

if __name__ == '__main__':
    JoyArmCommander()
    rospy.spin()
