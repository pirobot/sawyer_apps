#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
import numpy as np

rospy.init_node('arm_tracker_publisher')

pub = rospy.Publisher('target_pose', PoseStamped, queue_size = 1)

hdr = Header(stamp=rospy.Time.now(), frame_id='base')
newPose = PoseStamped(
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

pub.publish(newPose)
rospy.sleep(1)
pub.publish(newPose)
rospy.sleep(3)

num_pts = 100
arr = np.arange(num_pts, dtype=np.float)
print(arr)
y_arr = 0.160220965428 + 0.2 * np.sin(2*np.pi*0.015*arr)
#y_arr = 0.160220965428 - 0.005 * arr
#x_arr = 0.5 + 0.2 * np.sin(2*np.pi*0.03*arr)

print(y_arr)

for i in range(num_pts):
    newPose.header.stamp = rospy.Time.now()
    newPose.pose.position.y = y_arr[i]
    #newPose.pose.position.x = x_arr[i]
    pub.publish(newPose)
    rospy.sleep(0.1)
# newPose.header.stamp = rospy.Time.now()
# newPose.pose.position.y = y_arr[i]
# pub.publish(newPose)
# rospy.sleep(1)
# i += 1

