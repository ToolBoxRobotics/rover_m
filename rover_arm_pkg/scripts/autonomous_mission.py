#!/usr/bin/env python3

import rospy
import actionlib
import sys
import moveit_commander
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class RoverMissionCommander:
    def __init__(self):
        rospy.init_node('rover_mission_commander')
        
        # --- 1. SETUP NAVIGATION CLIENT ---
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for Move Base server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Navigation Ready.")

        # --- 2. SETUP ARM COMMANDER ---
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        
        # Set tolerances
        self.arm_group.set_planning_time(5.0)
        
        rospy.loginfo("--- MISSION CONTROL ONLINE ---")

    def go_to_waypoint(self, x, y, yaw_degrees):
        """ Sends a GPS-like coordinate to the navigation stack """
        rospy.loginfo(f"Navigating to: X={x}, Y={y}...")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" # or "odom" if no map yet
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Convert Yaw (Degrees) to Quaternion
        q = quaternion_from_euler(0, 0, yaw_degrees * 0.0174533)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.nav_client.send_goal(goal)
        success = self.nav_client.wait_for_result()
        
        if success:
            rospy.loginfo("Arrived at Waypoint.")
        else:
            rospy.logwarn("Failed to reach Waypoint!")
        return success

    def set_arm_pose(self, name):
        """ Moves to a pre-defined pose (home, stow) """
        rospy.loginfo(f"Moving Arm to: {name}")
        self.arm_group.set_named_target(name)
        success = self.arm_group.go(wait=True)
        if not success:
            rospy.logwarn(f"Arm failed to reach {name}")
        return success

    def set_gripper(self, state):
        """ state: 'open' or 'closed' """
        rospy.loginfo(f"Gripper: {state}")
        if state == "open":
            val = 1.0
        else:
            val = 0.0
        self.hand_group.set_joint_value_target("gripper_joint", val)
        self.hand_group.go(wait=True)
        rospy.sleep(1.0) # Wait for servo

    def perform_grasp_sequence(self):
        """ 
        Blind Grasp Sequence:
        1. Open Gripper
        2. Dip down (Cartesian)
        3. Close Gripper
        4. Lift up
        """
        rospy.loginfo("Starting Grasp Sequence...")
        
        # 1. Open
        self.set_gripper("open")
        
        # 2. Cartesian Move Down (Z -5cm)
        rospy.loginfo("Approaching object...")
        current_pose = self.arm_group.get_current_pose().pose
        waypoints = []
        
        # Target: 5cm lower
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation = current_pose.orientation
        wpose.position = current_pose.position
        wpose.position.z -= 0.05 
        waypoints.append(wpose)
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        rospy.sleep(1.0)
        
        # 3. Close
        self.set_gripper("closed")
        
        # 4. Lift Up (Z +10cm)
        rospy.loginfo("Lifting object...")
        waypoints = []
        wpose.position.z += 0.10 
        waypoints.append(wpose)
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)

    def run(self):
        # --- PHASE 1: INITIALIZE ---
        self.set_arm_pose("stow") # Ensure safe for driving
        
        # --- PHASE 2: TRAVEL TO TARGET ---
        # Navigate 2 meters forward
        if not self.go_to_waypoint(2.0, 0.0, 0):
            return

        # --- PHASE 3: DEPLOY ---
        self.set_arm_pose("home") # Unfold
        
        # Note: Here you would typically insert a Vision call:
        # coords = detect_object()
        # if coords: adjust_position(coords)
        
        # --- PHASE 4: ACTION ---
        self.perform_grasp_sequence()
        
        # --- PHASE 5: RETURN HOME ---
        self.set_arm_pose("stow") # Secure the "sample"
        
        # Return to start, but turn around (180 deg) to park backwards?
        # Let's just drive back to 0,0
        self.go_to_waypoint(0.0, 0.0, 180)
        
        rospy.loginfo("MISSION COMPLETE. SHUTTING DOWN.")

if __name__ == '__main__':
    try:
        mission = RoverMissionCommander()
        mission.run()
    except rospy.ROSInterruptException:
        pass
