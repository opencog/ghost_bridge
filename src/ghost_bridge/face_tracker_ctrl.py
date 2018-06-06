import tf
import rospy
from blender_api_msgs.msg import Target


class FaceTracker(object):

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.tf = tf.TransformListener()
        self.eye_speed = 0.7
        self.head_speed = 0.5

        # Publishers for making the face and eyes look at a point
        self.face_target_pub = rospy.Publisher("/blender_api/set_face_target", Target, queue_size=1)
        self.gaze_target_pub = rospy.Publisher("/blender_api/set_gaze_target", Target, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            print("face tracking before")
            if self.tf.frameExists("/base_link") and self.tf.frameExists("/blender"):
                print("face tracking inside")
                t = self.tf.getLatestCommonTime("/closest_face", "/blender")
                position, quaternion = self.tf.lookupTransform("/closest_face", "/blender", t)
                x = position.x
                y = position.y
                z = position.z
                self.point_eyes_at_point(x, y, z, self.eye_speed)
                self.face_toward_point(x, y, z, self.head_speed)

            self.rate.sleep()

    def point_eyes_at_point(self, x, y, z, speed):
        """  Turn the robot's eyes towards the given target point

        :param float x: metres forward
        :param float y: metres to robots left
        :param float z:
        :param float speed:
        :return: None
        """

        msg = Target()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.speed = speed

        self.gaze_target_pub.publish(msg)
        rospy.logdebug("published gaze_at(x={}, y={}, z={}, speed={})".format(x, y, z, speed))

    def face_toward_point(self, x, y, z, speed):
        """ Turn the robot's face towards the given target point.

        :param float x: metres forward
        :param float y: metres to robots left
        :param float z:
        :param float speed:
        :return: None
        """

        msg = Target()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.speed = speed

        self.face_target_pub.publish(msg)
        rospy.logdebug("published face_(x={}, y={}, z={}, speed={})".format(x, y, z, speed))
