#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from std_msgs.msg import Float32MultiArray

class ArmBridgeNode:
    def __init__(self):
        rospy.init_node('arduino_arm_bridge')

        # --- CONFIGURATION ---
        # IMPORTANT: These names must match EXACTLY what is in your URDF
        # and be in the order the Arduino expects: [J1, J2, J3, J4, J5]
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        
        # Publisher to Arduino
        self.arduino_pub = rospy.Publisher('/arm_joint_goals', Float32MultiArray, queue_size=10)
        
        # Action Server for MoveIt
        # This makes the node look like a standard ROS controller to MoveIt
        self.server = actionlib.SimpleActionServer(
            '/arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction, 
            self.execute_cb, 
            auto_start=False
        )
        self.server.start()
        
        rospy.loginfo("Arm Bridge Ready. Waiting for MoveIt commands...")

    def execute_cb(self, goal):
        success = True
        
        # 1. Extract the trajectory points
        # MoveIt sends a list of points (path). 
        # Since we use AccelStepper on Arduino for smooth motion, 
        # we strictly grab the FINAL point (the destination).
        if not goal.trajectory.points:
            rospy.logwarn("Received empty trajectory")
            self.server.set_aborted()
            return

        last_point = goal.trajectory.points[-1]
        target_positions = last_point.positions
        
        # 2. Map Joint Names to Array Indices
        # goal.trajectory.joint_names might be in random order. 
        # We must reorder the data to match self.joint_names (J1->J5).
        ordered_goals = [0.0] * 5
        
        try:
            for i, name in enumerate(self.joint_names):
                # Find where 'joint1' is in the incoming message
                index_in_msg = goal.trajectory.joint_names.index(name)
                # Grab the position value from that index
                ordered_goals[i] = target_positions[index_in_msg]
        except ValueError as e:
            rospy.logerr(f"Joint mismatch! MoveIt sent names: {goal.trajectory.joint_names}")
            self.server.set_aborted()
            return

        # 3. Publish to Arduino
        msg = Float32MultiArray()
        msg.data = ordered_goals
        
        # Add Gripper placeholder if needed (set to 0.0 or handle separately)
        # msg.data.append(0.0) 
        
        self.arduino_pub.publish(msg)
        
        rospy.loginfo(f"Sending to Arduino: {ordered_goals}")

        # 4. Wait for execution (Simulated)
        # Ideally, the Arduino would report back when done.
        # Here, we just wait a bit based on the time_from_start of the trajectory
        execution_time = last_point.time_from_start.to_sec()
        rospy.sleep(execution_time)

        self.server.set_succeeded(FollowJointTrajectoryResult())

if __name__ == '__main__':
    try:
        node = ArmBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
