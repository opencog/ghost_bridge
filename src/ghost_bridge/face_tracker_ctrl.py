import tf
import rospy
import math
from blender_api_msgs.msg import Target
from ghost_bridge.srv import GazeFocus, GazeFocusResponse


class FaceTracker(object):

    DIST_THRESH = 0.0

    def __init__(self):
        self.rate = rospy.Rate(2)
        self.tf = tf.TransformListener()
        self.eye_speed = 0.2
        self.head_speed = 0.7

        self.face_frame = "closest_face"
        self.blender_frame = "blender"
        self.idle_frame = "audience"
        self.last_position = None

        # Publishers for making the face and eyes look at a point
        self.face_target_pub = rospy.Publisher("/blender_api/set_face_target", Target, queue_size=1)
        self.gaze_target_pub = rospy.Publisher("/blender_api/set_gaze_target", Target, queue_size=1)

        # kick off the face frame setting service
        self.set_gaze_srv = rospy.Service('set_gaze_focus', GazeFocus, self.handle_set_gaze_focus)

    def run(self):
        while not rospy.is_shutdown():
            if self.tf.frameExists(self.face_frame) and self.tf.frameExists(self.blender_frame):
                t = self.tf.getLatestCommonTime(self.face_frame, self.blender_frame)
                position, quaternion = self.tf.lookupTransform(self.blender_frame, self.face_frame, t)
            elif self.tf.frameExists(self.idle_frame) and self.tf.frameExists(self.blender_frame):
                position, quaternion = self.tf.lookupTransform(self.blender_frame, self.idle_frame)
            else:
                self.rate.sleep()

            x = position[0]
            y = position[1]
            z = position[2]

            update_target = self.last_position is None

            if self.last_position is not None:
                dist = FaceTracker.distance(position, self.last_position)
                update_target = dist > FaceTracker.DIST_THRESH

            if update_target:
                self.point_eyes_at_point(x, y, z, self.eye_speed)
                self.face_toward_point(x, y, z, self.head_speed)
                self.last_position = position

            self.rate.sleep()

    @staticmethod
    def distance(p1, p2):
        return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))

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

    def handle_set_gaze_focus(self, req):
        self.face_frame = req.face_frame
        self.head_speed = req.speed

        return GazeFocusResponse()
