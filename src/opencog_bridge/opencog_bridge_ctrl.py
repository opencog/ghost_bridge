import rospy
from hr_msgs.msg import ChatMessage
from ros_people_model.msg import Faces

from opencog_bridge.perception_ctrl import PerceptionCtrl

'''
Subscribes to topics published by
    https://github.com/hansonrobotics/asr/blob/master/scripts/google_speech.py
and forwards them to OpenCog as per
    https://github.com/opencog/opencog/tree/master/opencog/ghost
'''


class OpenCogBridge:
    EMOTION_MAP = {
        0: "anger",
        1: "disgust",
        2: "fear",
        3: "happy",
        4: "sad",
        5: "surprise",
        6: "neutral"
    }

    def __init__(self):
        self.perception_ctrl = PerceptionCtrl()
        robot_name = rospy.get_param("robot_name")
        rospy.Subscriber(robot_name + "/words", ChatMessage, self.perceived_word)
        rospy.Subscriber(robot_name + "/speech", ChatMessage, self.perceived_sentence)
        rospy.Subscriber('/faces', Faces, self.faces_cb)

    def perceived_word(self, msg):
        self.perception_ctrl.perceived_word(msg.utterance)

    def perceived_sentence(self, msg):
        self.perception_ctrl.perceived_sentence(msg.utterance)

    def faces_cb(self, data):
        for face in data.faces:
            if face.face_id is "":
                continue

            self.perception_ctrl.perceived_face(face.face_id, face.position.x, face.position.y, face.position.z)

            if len(face.emotions) > 0:
                for i, strength in enumerate(face.emotions):
                    self.perception_ctrl.perceived_emotion(face.face_id, OpenCogBridge.EMOTION_MAP[i], strength)
