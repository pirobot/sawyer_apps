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
import sys, os
import moveit_commander
from geometry_msgs.msg import PoseStamped

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper'

GRIPPER_JOINT_NAMES = ['right_gripper_endpoint']

REFERENCE_FRAME = 'base'

class ArmTracker:
    def __init__(self):
        rospy.init_node('arm_tracker')
        
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.on_shutdown(self.shutdown)
        
        # Initialize the move group for the right arm
        self.right_arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        
        self.right_arm.set_planner_id("RRTkConfigDefault")
        
        # Keep track of the last target pose
        self.last_target_pose = PoseStamped()
        
        # Set the right arm reference frame
        self.right_arm.set_pose_reference_frame(REFERENCE_FRAME)
                        
        # Allowing replanning slows things down
        self.right_arm.allow_replanning(False)
                
        # Set a position tolerance in meters
        self.right_arm.set_goal_position_tolerance(0.01)
        
        # Set an orientation tolerance in radians
        self.right_arm.set_goal_orientation_tolerance(0.05)
                
        #self.right_arm.set_planning_time(0.015)
        
        # Set max acceleration and velocity scaling (0-1)
        self.right_arm.set_max_acceleration_scaling_factor(1.0)
        
        self.right_arm.set_max_velocity_scaling_factor(1.0)
        
        # What is the end effector link?
        self.end_effector_link = self.right_arm.get_end_effector_link()
        
        # Track how many times are given a request
        self.request_count = 0
        
        # Track how many requests are successful
        self.move_count = 0
        
        self.cartesian = True
        
        # Use queue_size=1 so we don't pile up outdated target messages
        self.target_subscriber = rospy.Subscriber('target_pose', PoseStamped, self.update_target_pose, queue_size=1)
        
        rospy.loginfo("Ready for action!")

                    
    def update_target_pose(self, target_pose):
        self.request_count += 1
        
        # Set the start state to the current state
        self.right_arm.set_start_state_to_current_state()
        
        success = False
        
        if self.cartesian:
            fraction = 0.0
            maxtries = 1
            attempts = 0
     
            # Plan the Cartesian path connecting the waypoints
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.right_arm.compute_cartesian_path (
                                        [target_pose.pose],   # target pose
                                        0.01,            # eef_step
                                        0.0,             # jump_threshold
                                        True)            # avoid_collisions
                
                # Increment the number of attempts 
                attempts += 1
                
                # Print out a progress message
                #if attempts % 10 == 0:
                #    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # If we have a complete plan, execute the trajectory
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
    
                success = self.right_arm.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

        else:
            # Set the goal pose to the target pose
            self.right_arm.set_pose_target(target_pose, self.end_effector_link)
    
            # Plan the trajectory to the goal
            #try:
            traj = self.right_arm.plan()
            #except:
                #rospy.loginfo("IK or planning failed!")
        
            #try:
            success = self.right_arm.execute(traj, wait=True)
            #except:
                #rospy.loginfo("Execution failed!")
            
        if success:
            self.move_count += 1

        rospy.loginfo("Requests: " + str(self.request_count) + " Successful: " + str(self.move_count))

           
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
    

    
    
