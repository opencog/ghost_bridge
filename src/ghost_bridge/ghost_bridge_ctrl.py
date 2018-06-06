from Queue import Queue, Empty

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

    TTS_STOP_SLEEP_TIME = 2.0

    def __init__(self):
        self.hostname = "localhost"
        self.port = 17001

        self.action_feedback_ctrl = ActionFeedbackCtrl(self.hostname, self.port)
        self.perception_ctrl = PerceptionCtrl(self.hostname, self.port)
        self.robot_name = rospy.get_param("robot_name")
        self.face_id = ""
        self.tts_speaking = False
        self.sr_continuous = True

        # max size of 1 so that we all ways have the latest ChatScript answer if there is one
        self.cs_fallback_queue = Queue(maxsize=1)

        self.tts_pub = rospy.Publisher(self.robot_name + "/tts", TTS, queue_size=1)

        rospy.Subscriber('/ghost_bridge/say', GhostSay, self.ghost_say_cb)
        rospy.Subscriber(self.robot_name + "/chatbot_responses", TTS, self.cs_say_cb)
        rospy.Subscriber(self.robot_name + "/speech_events", String, self.tts_say_cb)
        rospy.Subscriber(self.robot_name + "/words", ChatMessage, self.perceive_word_cb)
        rospy.Subscriber(self.robot_name + "/speech", ChatMessage, self.perceive_sentence_cb)
        rospy.Subscriber('/faces_throttled', Faces, self.faces_cb)

        self.dynamic_reconfigure_srv = Server(GhostBridgeConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.sr_continuous = config['sr_continuous']
        rospy.logdebug("Dynamic reconfigure callback result: {0}".format(config))
        return config

    def tts_say_cb(self, msg):
        if msg.data == "start":
            self.tts_speaking = True
            self.action_feedback_ctrl.say_started()
        elif msg.data == "stop":
            if not self.sr_continuous:
                rospy.sleep(GhostBridge.TTS_STOP_SLEEP_TIME)
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

        if msg.fallback_id == "chatscript":
            try:
                # wait for three seconds if no new response to give ChatScript a chance.
                cs_fallback_text = self.cs_fallback_queue.get(True, 3)
            except Empty:
                cs_fallback_text = ""
                rospy.logwarn("cs_fallback_text is ''")
            self.publish_tts(cs_fallback_text)
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
        if self.sr_continuous or not self.tts_speaking:
            self.perception_ctrl.perceive_sentence(self.face_id, msg.utterance)
            self.perception_ctrl.perceive_face_talking(self.face_id, 0.0)
        else:
            rospy.logdebug("suppressing sentence perceived to GHOST")

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
