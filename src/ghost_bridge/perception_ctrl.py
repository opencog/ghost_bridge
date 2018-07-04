#
# atomic_msgs.py - Send data to the cogserver/atomspace.
#
# Copyright (C) 2015,2016,2017  Linas Vepstas
# Copyright (C) 2016,2017  Hanson Robotics
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

from ghost_bridge.netcat import netcat


# The code here is a quick, cheap hack to place information into the
# cogserver atomspace. It opens a socket to the cogserver, and sends
# scheme snippets across.  These are usually some Atomese.

class PerceptionCtrl:

    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port

    def perceive_face(self, face_id, x, y, z, confidence):
        """ Perceive a face

        :param str face_id: the id of the face
        :param float x:
        :param float y:
        :param float z:
        :param float confidence:
        :return: None
        """

        content = '(perceive-face "{}" {})\n'.format(face_id, confidence)
        netcat(self.hostname, self.port, content)

    def perceive_emotion(self, face_id, emotion_id, confidence):
        """ Perceive an emotion

        :param str face_id: the id of the face
        :param str emotion_id: the id of the perceived emotion
        :param float confidence: the confidence of the emotion from 0.0 to 1.0
        :return: None
        """

        content = '(perceive-emotion "{}" "{}" {})\n'.format(face_id, emotion_id, confidence)
        netcat(self.hostname, self.port, content)

    def perceive_eye_state(self, face_id, eye_id, state):
        """ Perceive the state of a person's eyes

        :param str face_id: the id of the face
        :param str eye_id: the id of the eye, either 'left' or 'right'
        :param float state: the state of the eye from 0.0 to 1.0. Fully closed is 0 and fully open is 1.0
        :return: None
        """

        content = '(perceive-eye-state "{}" "{}" {})\n'.format(face_id, eye_id, state)
        netcat(self.hostname, self.port, content)

    def perceive_face_talking(self, face_id, confidence):
        """ Perceive the state of a person's eyes

        :param str face_id: the id of the face
        :param float confidence: confidence of the face talking from 0.0 to 1.0
        :return: None
        """

        content = '(perceive-face-talking "{}" {})\n'.format(face_id, confidence)
        netcat(self.hostname, self.port, content)

    def perceive_word(self, face_id, word):
        """ Perceive an individual word that is a part of the sentence a person is currently speaking

        :param str face_id: the id of the face
        :param str word: the perceived word
        :return: None
        """

        content = '(perceive-word "{}" "{}")\n'.format(face_id, word)
        netcat(self.hostname, self.port, content)

    def perceive_sentence(self, face_id, sentence):
        """ Perceive the whole sentence after the user has finished speaking

        :param str face_id: the id of the face
        :param str sentence: the perceived sentence
        :return: None
        """

        content = '(ghost "{}")\n'.format(sentence)
        netcat(self.hostname, self.port, content)

    def perceive_neck_direction(self, direction):
        content = '(perceive-neck-dir "{}")\n'.format(direction)
        netcat(self.hostname, self.port, content)