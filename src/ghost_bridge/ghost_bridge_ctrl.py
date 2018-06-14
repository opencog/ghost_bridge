from Queue import Queue, Empty

import time
import re
import rospy
from dynamic_reconfigure.server import Server
from ghost_bridge.action_feedback_ctrl import ActionFeedbackCtrl
from ghost_bridge.cfg import GhostBridgeConfig
from ghost_bridge.msg import GhostSay
from ghost_bridge.perception_ctrl import PerceptionCtrl
from hr_msgs.msg import ChatMessage
from hr_msgs.msg import TTS
from ros_people_model.msg import Faces
from std_msgs.msg import String
from blender_api_msgs.msg import SetGesture
import numpy as np

class GhostBridge:
    EMOTION_MAP = {
        0: "anger",
        1: "disgust",
        2: "fear",
        3: "happy",
        4: "sad",
        5: "surprise",
        6: "neutral"
    }

    EYE_MAP = {
        0: "left",
        1: "right"
    }


    def __init__(self):
        self.hostname = "localhost"
        self.port = 17001

        self.action_feedback_ctrl = ActionFeedbackCtrl(self.hostname, self.port)
        self.perception_ctrl = PerceptionCtrl(self.hostname, self.port)
        self.robot_name = rospy.get_param("robot_name")
        self.face_id = ""
        self.tts_speaking = False
        self.sr_continuous = True
        self.sr_tts_timeout = 0.0
        self.refractory_block = False

        self.timer = None
        self.refractimer = None
        self.input_buffer = ""
        self.stt_cutoff_time = 0.0

        # max size of 1 so that we all ways have the latest ChatScript answer if there is one
        self.cs_fallback_queue = Queue(maxsize=1)

        self.tts_pub = rospy.Publisher(self.robot_name + "/tts", TTS, queue_size=1)
        self.gesture_pub = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=1)
        rospy.Subscriber('/ghost_bridge/say', GhostSay, self.ghost_say_cb)
        rospy.Subscriber(self.robot_name + "/chatbot_responses", TTS, self.cs_say_cb)
        rospy.Subscriber(self.robot_name + "/speech_events", String, self.tts_say_cb)
        rospy.Subscriber(self.robot_name + "/words", ChatMessage, self.perceive_word_cb)
        rospy.Subscriber(self.robot_name + "/speech", ChatMessage, self.perceive_sentence_cb)
        rospy.Subscriber('/faces_throttled', Faces, self.faces_cb)

        self.dynamic_reconfigure_srv = Server(GhostBridgeConfig, self.dynamic_reconfigure_callback)


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




    def dynamic_reconfigure_callback(self, config, level):
        self.sr_continuous = config['sr_continuous']
        self.sr_tts_timeout = config['sr_tts_timeout']
        rospy.logdebug("Dynamic reconfigure callback result: {0}".format(config))
        return config

    def tts_say_cb(self, msg):
        if msg.data == "start":
            self.tts_speaking = True
            self.action_feedback_ctrl.say_started()
        elif msg.data == "stop":
            if not self.sr_continuous:
                rospy.sleep(self.sr_tts_timeout)
            self.tts_speaking = False
            self.action_feedback_ctrl.say_finished()

    def cs_say_cb(self, msg):
        rospy.logdebug("cs_fallback_text: '{}'".format(msg.text))
        # Empty the queue and put new answer.
        while not self.cs_fallback_queue.empty():
            self.cs_fallback_queue.get_nowait()
        self.cs_fallback_queue.put(msg.text)

    def ghost_say_cb(self, msg):
        rospy.logdebug("ghost_say_cb: '{}', '{}'".format(msg.text, msg.fallback_id))
        self.refractory_block = False
        if msg.fallback_id == "chatscript":
            try:
                # wait for three seconds if no new response to give ChatScript a chance.
                cs_fallback_text = self.cs_fallback_queue.get(True, 3)
            except Empty:
                cs_fallback_text = ""
                rospy.logwarn("cs_fallback_text is ''")
            self.publish_tts(cs_fallback_text)
            self.gesture("think-l",1.0,0.8,1)
        else:
            self.publish_tts(msg.text)

    def publish_tts(self, text):
        regex = '{% set delay=([0-9]+) %}'
        reg_result = re.compile(regex).findall(text)
        if reg_result:
          #print("---> regex caught")
          self.stt_cutoff_time = int(reg_result[0])
          self.timer = None
          rospy.logdebug("setting delay to '{}'".format(self.stt_cutoff_time))
        re.match('{% set delay=[0-9]+ %}', text)
        msg = TTS()
        msg.text = re.sub(regex, '', text)
        msg.lang = 'en-US'
        self.tts_pub.publish(msg)
        rospy.logdebug("published tts: '{}', '{}'".format(msg.text, msg.lang))

    def perceive_word_cb(self, msg):
        self.perception_ctrl.perceive_word(self.face_id, msg.utterance)
        self.perception_ctrl.perceive_face_talking(self.face_id, 1.0)
        if np.random.randint(4) == 3:
            self.gesture("nod-1",0.9,np.random.random()*0.6,1)
        self.reset_timer()

    def perceive_sentence_cb(self, msg):
        #print("perceive sentence cb")
        if self.sr_continuous or not self.tts_speaking:
            # This should be append, but subverting it here because of ghost bug
            self.input_buffer = msg.utterance
            #self.input_buffer += msg.utterance
            self.reset_timer()
            if self.stt_cutoff_time > 0.0:
                self.gesture("nod-2",0.8,0.85,1)
        else:
            rospy.logwarn("suppressing sentence perceived to GHOST")

    def send_perceived_sentence(self, evt):
        #print("send perceived sentence called")
        if self.input_buffer and not self.refractory_block:
          tosend = self.input_buffer
          self.input_buffer = ""
          #block perception for a while to fix blocking
          self.refractory_block = True
          if self.refractimer is not None:
              self.refractimer.shutdown()
              self.refractimer = None
          self.refractimer = rospy.Timer(rospy.Duration(10.0), self.reset_refractory, oneshot=True)
          rospy.logdebug("(ghost '{}')".format(tosend))
          self.perception_ctrl.perceive_sentence(self.face_id, tosend[:64])
          self.perception_ctrl.perceive_face_talking(self.face_id, 0.0)

    def reset_refractory(self, evt):
        self.refractory_block = False

    def reset_timer(self):
        #print("--> Reset timer called")
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
        self.timer = rospy.Timer(rospy.Duration(self.stt_cutoff_time + 0.0001), self.send_perceived_sentence, oneshot=True)
        #self.timer.start()

    def faces_cb(self, data):
        for face in data.faces:
            self.perception_ctrl.perceive_face(face.face_id, face.position.x, face.position.y, face.position.z,
                                               face.certainty)

            if len(face.eye_states) > 0:
                for i, state in enumerate(face.eye_states):
                    self.perception_ctrl.perceive_eye_state(face.face_id, GhostBridge.EYE_MAP[i], state)

            if len(face.emotions) > 0:
                for i, confidence in enumerate(face.emotions):
                    self.perception_ctrl.perceive_emotion(face.face_id, GhostBridge.EMOTION_MAP[i], confidence)
