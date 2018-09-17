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

import rospy
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import BlinkCycle
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SaccadeCycle
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import SomaState
from ghost_bridge.msg import GhostSay
from blender_api_msgs.srv import SetParam
from std_msgs.msg import String

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
    BPY_PARAM_SACCADE = "bpy.data.scenes[\"Scene\"].actuators.ACT_saccade.HEAD_PARAM_enabled"
    BPY_PARAM_BLINK = "bpy.data.scenes[\"Scene\"].actuators.ACT_blink_randomly.HEAD_PARAM_enabled"

    def __init__(self):
        # The below will hang until roscore is started!
        rospy.loginfo("Starting ghost_bridge_actions node")
        rospy.init_node("ghost_bridge_actions", log_level=rospy.DEBUG)

        # Publishers for making facial expressions and gestures
        self.emotion_state_pub = rospy.Publisher("/blender_api/set_emotion_state", EmotionState, queue_size=1)
        self.emotion_value_pub = rospy.Publisher("/blender_api/set_emotion_value", EmotionState, queue_size=1)
        self.gesture_pub = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=1)
        self.soma_pub = rospy.Publisher("/blender_api/set_soma_state", SomaState, queue_size=2)
        self.blink_pub = rospy.Publisher("/blender_api/set_blink_randomly", BlinkCycle, queue_size=1)
        self.saccade_pub = rospy.Publisher("/blender_api/set_saccade", SaccadeCycle, queue_size=1)

        # Publisher for controlling text to speech, i.e. making text to speech stop
        self.robot_name = rospy.get_param("robot_name")
        self.tts_control_pub = rospy.Publisher(self.robot_name + '/tts_control', String, queue_size=1)

        # Text to speech publisher
        self.ghost_tts_pub = rospy.Publisher("/ghost_bridge/say", GhostSay, queue_size=1)

        # blender set param
        self.blender_set_param_srv = rospy.ServiceProxy('/blender_api/set_param', SetParam)

        # Dictionary of components with controllable parameters. It is structured as
        # {component : { paramater : value }}
        self.component_parameters = {}
        # Speech output parameters based on https://www.w3.org/TR/speech-synthesis/
        self.component_parameters["speech"] = {}
        self.component_parameters["speech"]["volume"] = 0
        self.component_parameters["speech"]["rate"] = 1

        # Subscribers to get the available emotions and gestures
        rospy.Subscriber("/blender_api/available_emotion_states", AvailableEmotionStates, self.get_emotions_cb)
        rospy.Subscriber("/blender_api/available_gestures", AvailableGestures, self.get_gestures_cb)

    def update_parameter(self, component, parameter, value):
        """Update the parameters of the robot

        :param str component: identifier of the robot component or function
        :param str parameter: identifier of the parameter
        :param float value: the value of the parameter
        :return: None
        """
        self.component_parameters[component][parameter] = value
        rospy.logdebug("Updated parameter {}-{}={}".format(component, parameter, value))

    def say(self, text, fallback_id):
        """ Make the robot vocalize text

        :param str text: the text to vocalize
        :param str fallback_id: the id of the engine to fallback too
        :return: None
        """
        ssml_template = "<prosody rate=\"{rate}\" volume=\"{volume}dB\"> {} </prosody>"
        ssml_text = ssml_template.format(text, **self.component_parameters["speech"])
        msg = GhostSay()
        msg.text = ssml_text
        msg.fallback_id = fallback_id

        self.ghost_tts_pub.publish(msg)
        rospy.logdebug("published say(text={}, fallback={})".format(text, fallback_id))

    def say_cancel(self):
        """ Stop the robot from vocalizing its current sentence

        :return: None
        """

        # TODO: the implementation for cancelling robot actions is bloody hacky, we should be using actionlib for
        # controlling robot actions. See here: http://wiki.ros.org/actionlib
        self.tts_control_pub.publish("shutup")

        rospy.logdebug("published shutup")

    def gaze_at(self, face_id, speed):
        rospy.logwarn("gaze_at: not implemented")

    def gaze_at_cancel(self):
        rospy.logwarn("gaze_at_cancel: not implemented")

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

    def blink_cancel(self):
        try:
            self.blender_set_param_srv(ActionCtrl.BPY_PARAM_BLINK, "False")
            rospy.logdebug("blink_cancel: blender_api/set_param service called")
        except rospy.ServiceException, e:
            rospy.logerr("blink_cancel: blender_api/set_param service call failed %s" % e)

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
            ctrl.saccade(0.8, 0.3, 1.0, 15.0, 100.0, 90.0, 27.0, 0.8, 0.2)  # Explore-the-room when not conversing
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

    def saccade_cancel(self):
        try:
            self.blender_set_param_srv(ActionCtrl.BPY_PARAM_SACCADE, "False")
            rospy.logdebug("saccade_cancel: blender_api/set_param service called")
        except rospy.ServiceException, e:
            rospy.logerr("saccade_cancel: blender_api/set_param service call failed %s" % e)

    def emote(self, name, magnitude, duration, blend):
        """ Set the robot's emotional state

        :param str name: the id of the emotion
        :param float magnitude: the magnitude of the emotion from 0.0 to 1.0
        :param float duration: the time in seconds that the emotion lasts for
        :param bool blend: blend the emotion with other emotions that also have blend=True. If an emotion is sent with
                           blend=False, then it will overwrite all previously sent and active blendable emotions.
        :return: None
        """
        msg = EmotionState()
        msg.name = name
        msg.magnitude = magnitude
        msg.duration.secs = int(duration)
        msg.duration.nsecs = 1000000000 * (duration - int(duration))

        if blend:
            self.emotion_value_pub.publish(msg)
        else:
            self.emotion_state_pub.publish(msg)
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

        :param str name: the id of the soma state, this can be one of 'normal', 'breathing', 'normal-saccades' and 'sleep'
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

    def soma_cancel(self, name):
        msg = SomaState()
        msg.name = name
        msg.ease_in.secs = 0
        msg.ease_in.nsecs = 0.1 * 1000000000
        msg.magnitude = 0.0
        msg.rate = 1

        self.soma_pub.publish(msg)
        rospy.logdebug("publish soma(name={}, magnitude={}, rate={}, ease_in={})".format(name, msg.magnitude, msg.rate,
                                                                                         msg.ease_in.secs))

        rospy.logdebug("soma_cancel: {}".format(name))

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
