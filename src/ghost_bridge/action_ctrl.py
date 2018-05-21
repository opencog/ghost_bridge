#
# action_ctrl.py - ROS messaging module for OpenCog behaviors.
# Copyright (C) 2015  Hanson Robotics
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License v3 as
# published by the Free Software Foundation and including the exceptions
# at http://opencog.org/wiki/Licenses
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program; if not, write to:
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

import logging

import numpy
import rospy
import tf
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import BlinkCycle
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SaccadeCycle
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import SomaState
from blender_api_msgs.msg import Target
from ghost_bridge.msg import GhostSay

logger = logging.getLogger('hr.ghost_bridge_actions')


# ROS interfaces for the Atomese (OpenCog) Behavior Tree. Publishes
# ROS messages for animation control (smiling, frowning), and subscribes
# to STT/TTS and chatbot messages.
#
# This is meant to be a convenience wrapper, allowing Eva to be
# controlled from OpenCog Atomese.  Although it probably works as
# a stand-alone ROS node, it was not designed to be used that way.
# In particular, the python interpreter built into the atomspace
# will be runnig this code.
#
# It currently handles both control messages (publishing of expression
# and gesture animations), as well as some sensory input (mostly
# STT, TTS and chatbot interactions).
#


class ActionCtrl:

    def __init__(self):
        # The below will hang until roscore is started!
        rospy.loginfo("Starting ghost_bridge_actions node")
        rospy.init_node("ghost_bridge_actions", log_level=rospy.DEBUG)

        # Obtain the blender-to-camera coordinate-frame conversion
        # matrix.  XXX FIXME This is some insane hack that does not
        # make sense.  All 3D coordinates are supposed to be in
        # head-centered coordinates, bot for the sensory subsystem,
        # and also for the look-at subsystem. However, someone
        # screwed something up somewhere, and now we hack around
        # it here. XXX This is really bad spaghetti-code programming.
        # Fuck.
        self.tf = tf.TransformListener()
        self.conv_mat = None
        self.get_conv_mat()

        # Publishers for making facial expressions and gestures
        self.expression_pub = rospy.Publisher("/blender_api/set_emotion_state", EmotionState, queue_size=1)
        self.gesture_pub = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=1)
        self.soma_pub = rospy.Publisher("/blender_api/set_soma_state", SomaState, queue_size=2)
        self.blink_pub = rospy.Publisher("/blender_api/set_blink_randomly", BlinkCycle, queue_size=1)
        self.saccade_pub = rospy.Publisher("/blender_api/set_saccade", SaccadeCycle, queue_size=1)

        # Publishers for making the face and eyes look at a point
        self.face_target_pub = rospy.Publisher("/blender_api/set_face_target", Target, queue_size=1)
        self.gaze_target_pub = rospy.Publisher("/blender_api/set_gaze_target", Target, queue_size=1)

        # Text to speech publisher
        self.ghost_tts_pub = rospy.Publisher("/ghost_bridge/say", GhostSay, queue_size=1)

        # Subscribers to get the available emotions and gestures
        rospy.Subscriber("/blender_api/available_emotion_states", AvailableEmotionStates, self.get_emotions_cb)
        rospy.Subscriber("/blender_api/available_gestures", AvailableGestures, self.get_gestures_cb)

    def get_conv_mat(self):
        # try to initialize conv mat if it doesn't exist
        if self.conv_mat is None:
            try:
                rospy.loginfo("Waiting for the camera-blender transform")
                self.tf.waitForTransform('camera', 'blender', rospy.Time(0), rospy.Duration(10))  # world
            except Exception:
                rospy.logerr("No camera transforms!")
                return None

            rospy.loginfo("Got the camera-blender transform")
            (trans, rot) = self.tf.lookupTransform('blender', 'camera', rospy.Time(0))
            a = tf.listener.TransformerROS()
            self.conv_mat = a.fromTranslationRotation(trans, rot)

        return self.conv_mat

    def say(self, text, fallback_id):
        """ Make the robot vocalize text

        :param str text: the text to vocalize
        :param str fallback_id: the id of the engine to fallback too
        :return: None
        """
        msg = GhostSay()
        msg.text = text
        msg.fallback_id = fallback_id

        self.ghost_tts_pub.publish(msg)
        rospy.logdebug("published say(text={}, fallback={})".format(text, fallback_id))

    def gaze_at(self, x, y, z, speed):
        """  Turn the robot's eyes towards the given target point

        :param float x: metres forward
        :param float y: metres to robots left
        :param float z:
        :param float speed:
        :return: None
        """

        conv_mat = self.get_conv_mat()
        if conv_mat is None:
            rospy.logerr("gaze_at: conv_mat not initialized")
            return

        xyz1 = numpy.array([x, y, z, 1.0])
        xyz = numpy.dot(conv_mat, xyz1)
        msg = Target()
        msg.x = xyz[0]
        msg.y = xyz[1]
        msg.z = xyz[2]
        msg.speed = speed

        self.gaze_target_pub.publish(msg)
        rospy.logdebug("published gaze_at(x={}, y={}, z={}, speed={})".format(x, y, z, speed))

    def face_toward(self, x, y, z, speed):
        """ Turn the robot's face towards the given target point.

        :param float x: metres forward
        :param float y: metres to robots left
        :param float z:
        :param float speed:
        :return: None
        """

        conv_mat = self.get_conv_mat()
        if conv_mat is None:
            rospy.logerr("face_toward: conv_mat not initialized")
            return

        xyz1 = numpy.array([x, y, z, 1.0])
        xyz = numpy.dot(conv_mat, xyz1)
        msg = Target()
        msg.x = xyz[0]
        msg.y = xyz[1]
        msg.z = xyz[2]
        msg.speed = speed

        self.face_target_pub.publish(msg)
        rospy.logdebug("published face_(x={}, y={}, z={}, speed={})".format(x, y, z, speed))

    def blink(self, mean, variation):
        """ Set the robot's blink cycle

        :param float mean: mean time in seconds between blinks
        :param float variation: a random deviation from the mean blink time in seconds
        :return: None
        """

        msg = BlinkCycle()
        msg.mean = mean
        msg.variation = variation

        self.blink_pub.publish(msg)
        rospy.logdebug("published blink(mean={}, variation={})".format(mean, variation))

    def saccade(self, mean, variation, paint_scale, eye_size, eye_distance, mouth_width, mouth_height, weight_eyes,
                weight_mouth):
        """ Set the robot's eye saccade cycle, i.e. how the eye's twitch and move around automatically.

        :param float mean:
        :param float variation:
        :param float paint_scale:
        :param float eye_size:
        :param float eye_distance:
        :param float mouth_width:
        :param float mouth_height:
        :param float weight_eyes:
        :param float weight_mouth:
        :return: None

        Examples:
            ctrl = ActionCtrl()
            ctrl.saccade(0.8, 0.3, 0.3, 15.0, 100.0, 90.0, 27.0, 0.8, 0.2)  # Explore-the-room when not conversing
            ctrl.saccade(0.8, 0.5, 0.3, 11.5, 100.0, 90.0, 5.0, 0.8, 0.2)   # Variation 1: study face being looked at
                                                                            # during conversation
            ctrl.saccade(1.0, 0.6, 0.3, 11.0,  80.0, 50.0, 13.0, 0.8, 0.2)  # Variation 2: study face being looked at
                                                                            #during conversation
        """

        msg = SaccadeCycle()
        msg.mean = mean
        msg.variation = variation
        msg.paint_scale = paint_scale
        msg.eye_size = eye_size
        msg.eye_distance = eye_distance
        msg.mouth_width = mouth_width
        msg.mouth_height = mouth_height
        msg.weight_eyes = weight_eyes
        msg.weight_mouth = weight_mouth

        self.saccade_pub.publish(msg)
        rospy.logdebug("published saccade(mean={}, variation={}, paint_scale={}, "
                       "eye_size={}, eye_distance={}, mouth_width={}, mouth_height={}, "
                       "weight_eyes={}, weight_mouth={})".format(mean, variation, paint_scale, eye_size, eye_distance,
                                                                 mouth_width, mouth_height, weight_eyes, weight_mouth))

    def emote(self, name, magnitude, duration):
        """ Set the robot's emotional state

        :param str name: the id of the emotion
        :param float magnitude: the magnitude of the emotion from 0.0 to 1.0
        :param float duration: the time in seconds that the emotion lasts for
        :return: None
        """
        msg = EmotionState()
        msg.name = name
        msg.magnitude = magnitude
        msg.duration.secs = int(duration)
        msg.duration.nsecs = 1000000000 * (duration - int(duration))

        self.expression_pub.publish(msg)
        rospy.logdebug("published emote(name={}, magnitude={}, duration={})".format(name, magnitude, duration))

    def gesture(self, name, speed, magnitude, repeat):
        """ Set a pose on the robot's face

        :param str name: the id of the gesture
        :param float speed:
        :param float magnitude: the magnitude of the gesture from 0.0 to 1.0
        :param int32 repeat: number of times to repeat
        :return: None
        """

        msg = SetGesture()
        msg.name = name
        msg.speed = speed
        msg.magnitude = magnitude
        msg.repeat = repeat

        self.gesture_pub.publish(msg)
        rospy.logdebug("published gesture(name={}, speed={}, magnitude={}, repeat={})".format(name, speed, magnitude,
                                                                                              repeat))

    def soma(self, name, magnitude, rate, ease_in):
        """ Sets the robot's background facial expressions

        :param str name: the id of the soma state
        :param float magnitude: the magnitude of the soma facial expression from 0.0 to 1.0
        :param float rate:
        :param float ease_in:
        :return: None

        Examples:
            ctrl = ActionCtrl()
            ctrl.soma('sleep', 1, 1, 3)  # Sleep
            ctrl.soma('normal', 0.1, 1, 3) # Wake up
        """

        msg = SomaState()
        msg.name = name
        msg.magnitude = magnitude
        msg.rate = rate
        msg.ease_in.secs = int(ease_in)
        msg.ease_in.nsecs = 1000000000 * (ease_in - int(ease_in))

        self.soma_pub.publish(msg)
        rospy.logdebug("publish soma(name={}, magnitude={}, rate={}, ease_in={})".format(name, magnitude, rate,
                                                                                         ease_in))

    def get_emotions_cb(self, msg):
        """ Log the available emotions

        :param blender_api_msgs.msg.AvailableEmotionStates msg: available emotional states
        :return: None
        """

        rospy.loginfo("Available emotions: {}".format(msg.data))

    def get_gestures_cb(self, msg):
        """ Log the available gestures

        :param blender_api_msgs.msg.AvailableGestures msg: available gestures
        :return: None
        """

        rospy.loginfo("Available gestures: {}".format(msg.data))
