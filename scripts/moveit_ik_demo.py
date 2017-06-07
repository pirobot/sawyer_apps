#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
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

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the right arm
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'base'
        
        # Set the right arm reference frame accordingly
        right_arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(False)
        
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.001)
        right_arm.set_goal_orientation_tolerance(0.05)
        
        # Store the start pose
        start_pose = right_arm.get_current_pose(end_effector_link)
               
        # Set a valid target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.742965472241
        target_pose.pose.position.y = -0.422098289107
        target_pose.pose.position.z = 0.0640495711657
        target_pose.pose.orientation.x = 0.0223802305756
        target_pose.pose.orientation.y = 0.704731586969
        target_pose.pose.orientation.z = -0.0148928235808
        target_pose.pose.orientation.w = 0.708964540308

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        target_pose = PoseStamped(
            header=hdr,
            pose=Pose(
                # CAN CHANGE ABSOLUTE POSITION
                position=Point(
                    x = 1.06089534794,
                    y = 0.158850291264,
                    z = 0.315691943473
                ),
                orientation=Quaternion(
                   x = 0.707,
                   y = 0.0,
                   z = 0.707,
                   w = 0.0
                ),
            ),
        )

        
        # Set the start state to the current state
        right_arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the target pose
        right_arm.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = right_arm.plan()
        
        # Execute the planned trajectory
        right_arm.execute(traj)
         
        # Shift the end-effector to the left 5cm
        traj_array = []
        for i in range(40):
            right_arm.shift_pose_target(1, -0.01, end_effector_link)
            traj = right_arm.plan()
            n_points = len(traj.joint_trajectory.points)

            #for i in range(n_points):
            #    traj.joint_trajectory.points[i].velocities = [5]*7

            rospy.loginfo(traj)
            traj_array.append(traj)    
            #right_arm.execute(traj)
#            right_arm.go()
            #rospy.sleep(0.01)

        for i in range(40):
            right_arm.execute(traj_array[i])
            rospy.sleep(0.1)
  

        # Rotate the end-effector 90 degrees
#        right_arm.shift_pose_target(3, -1.57, end_effector_link)
#        right_arm.go()
#        rospy.sleep(1)
          
        # Go back to the original target pose
#        right_arm.set_pose_target(target_pose, end_effector_link)
#        right_arm.go()
#        rospy.sleep(1)
        
        # Go back to start pose
#        right_arm.set_start_state_to_current_state()
#        right_arm.set_pose_target(start_pose, end_effector_link)
#        right_arm.go()
#        rospy.sleep(1)
           
        # Finish up in the resting position  
        #right_arm.set_named_target('rest')
        #right_arm.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
