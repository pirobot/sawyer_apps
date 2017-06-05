#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

BUTTON_A      = 0
BUTTON_B      = 1
BUTTON_X      = 2
BUTTON_Y      = 3
BUTTON_LB     = 4
BUTTON_RB     = 5
BUTTON_BACK   = 6
BUTTON_START  = 7
BUTTON_POWER  = 8
BUTTON_LSTICK = 9
BUTTON_RSTICK = 10
BUTTON_DPAD_L = 11
BUTTON_DPAD_R = 12
BUTTON_DPAD_U = 13
BUTTON_DPAD_D = 14

AXIS_LR_LSTICK = 0
AXIS_UD_LSTICK = 1
AXIS_LR_RSTICK = 2
AXIS_UD_RSTICK = 3

class XBoxPose:
    def __init__(self):
        rospy.init_node('xbox_to_pose')

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base'
        self.target_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=5)
        
        self.max_x = 1.0
        self.max_y = 0.85
        self.max_z = 1.0
        
        self.last_pose_time = rospy.Time.now().to_sec()


    def xbox_to_pose(self, joy_message):
        self.target_pose.pose.position.x = (joy_message.axes[AXIS_UD_LSTICK] * self.max_x)
        self.target_pose.pose.position.y = (joy_message.axes[AXIS_LR_LSTICK] * self.max_y)
        self.target_pose.pose.position.z = (joy_message.axes[AXIS_UD_RSTICK] * self.max_z)
        self.target_pose.pose.orientation.w = 1.0

        if ((rospy.Time.now().to_sec() - self.last_pose_time) > 0.1):
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pub.publish(self.target_pose)
            self.last_pose_time = rospy.Time.now().to_sec()   

    def xbox_joy_receive(self):
        rospy.Subscriber('/joy', Joy, self.xbox_to_pose)
        print "xbox_to_pose node started"
        rospy.spin()

if __name__ == "__main__":
    xbox_to_pose = XBoxPose()
    xbox_to_pose.xbox_joy_receive()
    
