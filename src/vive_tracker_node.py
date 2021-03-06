#!/usr/bin/env python
import libsurvive
import sys
import rospy
from geometry_msgs.msg import Pose 

if __name__ == '__main__':
	rospy.init_node('vive_poser', anonymous=False)
	pub1 = rospy.Publisher('vive_pose', Pose, queue_size=10)
	actx = libsurvive.SimpleContext(sys.argv)

	for obj in actx.Objects():
	    print(obj.Name())
	while not rospy.is_shutdown() and actx.Running():
		    updated = actx.NextUpdated()
		    if updated:
			temp = Pose()
			poseandori, idk = updated.Pose()
			temp.position.x = poseandori.Pos[0]
			temp.position.y = poseandori.Pos[1]
			temp.position.z = poseandori.Pos[2]
			temp.orientation.x = poseandori.Rot[0]
			temp.orientation.y = poseandori.Rot[1]
			temp.orientation.z = poseandori.Rot[2]
			temp.orientation.w = poseandori.Rot[3]
			pub1.publish(temp)

