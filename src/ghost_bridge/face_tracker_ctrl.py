import tf
import rospy
import math
from blender_api_msgs.msg import Target
from ghost_bridge.msg import GazeAction, GazeActionFeedback
from actionlib import SimpleActionServer


class FaceTracker(object):
    DIST_THRESH = 0.0

    def __init__(self):
        self.rate = rospy.Rate(2)
        self.tf = tf.TransformListener()
        self.eye_speed = 0.2
        self.head_speed = 0.7

        self.blender_frame = "blender"
        self.default_position = [1, 0, 0]  # Looking straight ahead 1 metre
        self.last_position = None

        # Publishers for making the face and eyes look at a point
        self.face_target_pub = rospy.Publisher("/blender_api/set_face_target", Target, queue_size=1)
        self.gaze_target_pub = rospy.Publisher("/blender_api/set_gaze_target", Target, queue_size=1)

        # Gaze action server
        self.action_srv = SimpleActionServer("/gaze_action", GazeAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_srv.start()

    def execute_cb(self, goal):
        rospy.loginfo("Target goal received: " + str(goal))
        target_frame = goal.target

        while not rospy.is_shutdown() and not self.action_srv.is_preempt_requested() and self.action_srv.is_active():
            if self.tf.frameExists(target_frame) and self.tf.frameExists(self.blender_frame):
                time = self.tf.getLatestCommonTime(target_frame, self.blender_frame)
                position, quaternion = self.tf.lookupTransform(self.blender_frame, target_frame, time)
                update_target = self.last_position is None

                if self.last_position is not None:
                    dist = FaceTracker.distance(position, self.last_position)
                    update_target = dist > FaceTracker.DIST_THRESH

                if update_target:
                    self.gaze_at_point(position)

            self.action_srv.publish_feedback(GazeActionFeedback())
            self.rate.sleep()

        # If gaze has been cancelled then set default position
        self.gaze_at_point(self.default_position)

    @staticmethod
    def distance(p1, p2):
        return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))

    def gaze_at_point(self, position):
        x = position[0]
        y = position[1]
        z = position[2]

        self.point_eyes_at_point(x, y, z, self.eye_speed)
        self.face_toward_point(x, y, z, self.head_speed)
        self.last_position = position

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
