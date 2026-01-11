#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from std_msgs.msg import String

class WebCommander:
    def __init__(self):
        rospy.init_node('web_commander')
        
        # MoveIt Setup
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        
        # Subscribe to simple string commands from the Web UI
        rospy.Subscriber("/web/command", String, self.callback)
        
        rospy.loginfo("Web Commander Ready. Listening on /web/command")

    def callback(self, msg):
        cmd = msg.data
        rospy.loginfo(f"Received Web Command: {cmd}")
        
        if cmd == "HOME":
            self.arm_group.set_named_target("home")
            self.arm_group.go(wait=False)
            
        elif cmd == "STOW":
            self.arm_group.set_named_target("stow")
            self.arm_group.go(wait=False)
            
        elif cmd == "OPEN":
            self.hand_group.set_joint_value_target("gripper_joint", 1.0)
            self.hand_group.go(wait=False)
            
        elif cmd == "CLOSE":
            self.hand_group.set_joint_value_target("gripper_joint", 0.0)
            self.hand_group.go(wait=False)

if __name__ == '__main__':
    WebCommander()
    rospy.spin()
