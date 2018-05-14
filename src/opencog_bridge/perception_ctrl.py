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

from opencog_bridge.netcat import netcat


# The code here is a quick, cheap hack to place information into the
# cogserver atomspace. It opens a socket to the cogserver, and sends
# scheme snippets across.  These areu usually some Atomese.
#
class PerceptionCtrl:

    def __init__(self):
        self.hostname = "localhost"
        self.port = 17001

    # Face postions in the space-server
    def perceived_face(self, faceid, xx, yy, zz):
        face = '(perceived-face ' + str(faceid) + ' ' + str(xx) + ' ' + ' ' + str(yy) + ' ' + str(zz) + ')' + "\n"
        netcat(self.hostname, self.port, face)

    def perceived_emotion(self, faceid, emotiontype, strength):
        # (perceived-face-happy "UUID" happy 0.4)
        face = '(perceived-emotion ' + str(faceid) + ' ' + str(emotiontype) + ' ' + str(strength) + ')' + "\n"
        netcat(self.hostname, self.port, face)

    # --------------------------------------------------------
    # Speech-to-text stuff
    def perceived_sentence(self, stt):
        spoke = "(ghost \"" + stt + "\")\n"
        netcat(self.hostname, self.port, spoke)

    def perceived_word(self, stt):
        spoke = "(perceived-word \"" + stt + "\")\n"
        netcat(self.hostname, self.port, spoke)
