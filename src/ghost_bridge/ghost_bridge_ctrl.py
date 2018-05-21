import time
from threading import Lock

import rospy
from ghost_bridge.netcat import netcat
from ghost_bridge.perception_ctrl import PerceptionCtrl
from hr_msgs.msg import ChatMessage
from hr_msgs.msg import TTS
from ros_people_model.msg import Faces
from ghost_bridge.msg import GhostSay


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

    START_AGENTS_CMD = "agents-start opencog::AFImportanceDiffusionAgent opencog::WAImportanceDiffusionAgent " \
                       "opencog::AFRentCollectionAgent opencog::WARentCollectionAgent"

    def __init__(self):
        self.hostname = "localhost"
        self.port = 17001

        self.perception_ctrl = PerceptionCtrl(self.hostname, self.port)
        self.robot_name = rospy.get_param("robot_name")
        self.face_id = ""
        self.cs_fallback_text = ""
        self.tts_lock = Lock()

        self.start_agents()

        self.tts_pub = rospy.Publisher(self.robot_name + "/tts", TTS, queue_size=1)
        rospy.Subscriber('/ghost_bridge/say', GhostSay, self.ghost_say_cb)
        rospy.Subscriber(self.robot_name + "/chatbot_responses", TTS, self.cs_say_cb)
        rospy.Subscriber(self.robot_name + "/words", ChatMessage, self.perceive_word_cb)
        rospy.Subscriber(self.robot_name + "/speech", ChatMessage, self.perceive_sentence_cb)
        rospy.Subscriber('/faces_throttled', Faces, self.faces_cb)

    def start_agents(self):
        rospy.loginfo("Starting agents")
        netcat(self.hostname, self.port, GhostBridge.START_AGENTS_CMD)
        rospy.loginfo("Starting agents finished")

    def cs_say_cb(self, msg):
        with self.tts_lock:
            rospy.logdebug("cs_fallback_text: '{}'".format(msg.text))
            self.cs_fallback_text = msg.text

    def ghost_say_cb(self, msg):
        rospy.logdebug("ghost_say_cb: '{}', '{}'".format(msg.text, msg.fallback_id))

        with self.tts_lock:
            if msg.fallback_id == "chatscript":
                if self.cs_fallback_text == '':
                    rospy.logwarn("cs_fallback_text is ''")
                self.publish_tts(self.cs_fallback_text)
            else:
                self.publish_tts(msg.text)

    def publish_tts(self, text):
        msg = TTS()
        msg.text = text
        msg.lang = 'en-US'
        self.tts_pub.publish(msg)
        rospy.logdebug("published tts: '{}', '{}'".format(msg.text, msg.lang))

    def perceive_word_cb(self, msg):
        self.perception_ctrl.perceive_word(self.face_id, msg.utterance)
        self.perception_ctrl.perceive_face_talking(self.face_id, 1.0)

    def perceive_sentence_cb(self, msg):
        self.perception_ctrl.perceive_sentence(self.face_id, msg.utterance)
        self.perception_ctrl.perceive_face_talking(self.face_id, 0.0)

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
