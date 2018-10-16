#! /usr/bin/env python

import rospy
from ghost_bridge import FaceTracker


if __name__ == "__main__":

    try:
        rospy.init_node('face_tracker', log_level=rospy.DEBUG)
        node = FaceTracker()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

    rospy.loginfo("Exit OpenCog bridge")