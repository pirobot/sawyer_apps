#!/usr/bin/env python

"""
    arm_tracker.py - Version 0.1 2014-01-14
    
    Move the arm to point to a target on the /target_pose topic
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper'

GRIPPER_JOINT_NAMES = ['right_gripper_endpoint']

REFERENCE_FRAME = 'base'

class ArmTracker:
    def __init__(self):
        rospy.init_node('arm_tracker')
        
        rospy.on_shutdown(self.shutdown)
        
        # Maximum distance of the target before the arm will lower
        self.max_target_dist = 1.2
        
        # Distance between the last target and the new target before we move the arm
        self.last_target_threshold = 0.01
        
        # Distance between target and end-effector before we move the arm
        self.target_ee_threshold = 0.025
        
        # Initialize the move group for the right arm
        self.right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        #right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Keep track of the last target pose
        self.last_target_pose = PoseStamped()
        
        # Set the right arm reference frame
        self.right_arm.set_pose_reference_frame(REFERENCE_FRAME)
                        
        # Allowing replanning slows things down
        self.right_arm.allow_replanning(False)
                
        # Set a position tolerance in meters
        self.right_arm.set_goal_position_tolerance(0.05)
        
        # Set an orientation tolerance in radians
        self.right_arm.set_goal_orientation_tolerance(0.2)
        
        # What is the end effector link?
        self.end_effector_link = self.right_arm.get_end_effector_link()
        
        # Set the gripper target to closed position using a joint value target
        #right_gripper.set_joint_value_target(GRIPPER_CLOSED)
         
        # Plan and execute the gripper motion
        #right_gripper.go()
        #rospy.sleep(1)
                
        # Subscribe to the target topic
        rospy.wait_for_message('target_pose', PoseStamped)
        
        # Use queue_size=1 so we don't pile up outdated target messages
        self.target_subscriber = rospy.Subscriber('target_pose', PoseStamped, self.update_target_pose, queue_size=1)
        
        rospy.loginfo("Ready for action!")

                    
    def update_target_pose(self, target_pose):        
        # Set the start state to the current state
        self.right_arm.set_start_state_to_current_state()
    
        # Set the goal pose to the target pose
        self.right_arm.set_pose_target(target_pose, self.end_effector_link)
    
        # Plan the trajectory to the goal
        #traj = self.right_arm.plan()
    
        # Execute the planned trajectory
        #self.right_arm.execute(traj)
        
        # Plan and execute the trajectory
        success = self.right_arm.go()
        
        if success:
            # Store the current target as the last target
            self.last_target_pose = target_pose
        
        # Pause a bit between motions to keep from locking up
        rospy.sleep(0.1)
           
    def shutdown(self):
        # Stop any further target messages from being processed
        self.target_subscriber.unregister()
        
        # Stop any current arm movement
        self.right_arm.stop()
        
        # Move to the resting position
        #self.right_arm.set_named_target('resting')
        #self.right_arm.go()
        
        os._exit(0) 

if __name__ == "__main__":
    try:
        ArmTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")
    

    
    