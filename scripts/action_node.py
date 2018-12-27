#
# actions.py - OpenCog python schema to control Eva.
#
# This is a wrapper around a ROS node that is able to control the Eva
# blender model. This wrapper is desined so that each function can be
# called by OpenCog, from a GroundedPredicateNode. The functions are
# simple, and correspond more-or-less directly to the Eva API: the
# blender API can be told to play the smile animation, or the blink
# animation, or frown, fall asleep, turn, look, and so on.
#
# Copyright (C) 2015  Linas Vepstas
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

import rospy
from opencog.atomspace import TruthValue
from ghost_bridge import ActionCtrl

# The ROS layer.
action_ctrl = ActionCtrl()


# Global functions, because that's what PythonEval expects.
# Would be great if PythonEval was fixed to work smarter, not harder.
#
# Must return TruthValue, since EvaluationLinks expect TruthValues.

def set_parameter(component_node, parameter_node, value_node):
    component = component_node.name
    parameter = parameter_node.name
    value = float(value_node.name)
    action_ctrl.set_parameter(component, parameter, value)
    return TruthValue(1, 1)

def say(text_node, fallback_id_node):
    text = text_node.name
    fallback_id = fallback_id_node.name
    rospy.logdebug("say(text='{}', fallback_id='{}')".format(text, fallback_id))
    action_ctrl.say(text, fallback_id)
    return TruthValue(1, 1)

def say_cancel():
    rospy.logdebug("say_cancel()")
    action_ctrl.say_cancel()
    return TruthValue(1, 1)

def gaze_at(face_id_node, speed_node):
    face_id = face_id_node.name
    speed = float(speed_node.name)
    rospy.logdebug("gaze_at(face_id={}, speed={})".format(face_id, speed))
    action_ctrl.gaze_at(face_id, speed)
    return TruthValue(1, 1)


def gaze_at_cancel():
    rospy.logdebug("gaze_at_cancel()")
    action_ctrl.gaze_at_cancel()
    return TruthValue(1, 1)


def blink(mean_node, variation_node):
    mean = float(mean_node.name)
    variation = float(variation_node.name)
    rospy.loginfo("blink(mean={}, variation={})".format(mean, variation))
    action_ctrl.blink(mean, variation)
    return TruthValue(1, 1)


def blink_cancel():
    rospy.logdebug("blink_cancel()")
    action_ctrl.blink_cancel()
    return TruthValue(1, 1)


def saccade(mean_node, variation_node, paint_scale_node, eye_size_node, eye_distance_node, mouth_width_node,
            mouth_height_node, weight_eyes_node, weight_mouth_node):
    mean = float(mean_node.name)
    variation = float(variation_node.name)
    paint_scale = float(paint_scale_node.name)
    eye_size = float(eye_size_node.name)
    eye_distance = float(eye_distance_node.name)
    mouth_width = float(mouth_width_node.name)
    mouth_height = float(mouth_height_node.name)
    weight_eyes = float(weight_eyes_node.name)
    weight_mouth = float(weight_mouth_node.name)
    rospy.logdebug("saccade(mean={}, variation={}, paint_scale={}, "
                   "eye_size={}, eye_distance={}, mouth_width={}, mouth_height={}, "
                   "weight_eyes={}, weight_mouth={})".format(mean, variation, paint_scale, eye_size, eye_distance,
                                                             mouth_width, mouth_height, weight_eyes, weight_mouth))
    action_ctrl.saccade(mean, variation, paint_scale, eye_size, eye_distance, mouth_width, mouth_height, weight_eyes,
                        weight_mouth)
    return TruthValue(1, 1)


def saccade_cancel():
    rospy.logdebug("saccade_cancel()")
    action_ctrl.saccade_cancel()
    return TruthValue(1, 1)


def emote(name_node, magnitude_node, duration_node, blend_node):
    name = name_node.name

    # ugly hack because blender animation names are different
    if name is "happy":
        name = "happy.001"

    if name is "worry":
        name = "irritated"

    if name is "smile":
        name = "happy.002"

    magnitude = float(magnitude_node.name)
    duration = float(duration_node.name)
    blend = blend_node.name == "True"
    rospy.logdebug("emote(name='{}', magnitude={}, duration={}, blend={})".format(name, magnitude, duration, blend))
    action_ctrl.emote(name, magnitude, duration, blend)
    return TruthValue(1, 1)


def gesture(name_node, speed_node, magnitude_node, repeat_node):
    name = name_node.name
    speed = float(speed_node.name)
    magnitude = float(magnitude_node.name)
    repeat = int(float(repeat_node.name))
    rospy.logdebug("gesture(name='{}', speed={}, magnitude={}, repeat={})".format(name, speed, magnitude, repeat))
    action_ctrl.gesture(name, speed, magnitude, repeat)
    return TruthValue(1, 1)


def soma(name_node, magnitude_node, rate_node, ease_in_node):
    name = name_node.name
    magnitude = float(magnitude_node.name)
    rate = float(rate_node.name)
    ease_in = float(ease_in_node.name)
    rospy.logdebug("soma(name='{}', magnitude={}, rate={}, ease_in={})".format(name, magnitude, rate, ease_in))
    action_ctrl.soma(name, magnitude, rate, ease_in)
    return TruthValue(1, 1)


def soma_cancel(name_node):
    name = name_node.name
    rospy.logdebug("soma_cancel(name='{}')".format(name))
    action_ctrl.soma_cancel(name)
    return TruthValue(1, 1)


def sing():
    rospy.logdebug("sing()")
    action_ctrl.sing()
    return TruthValue(1, 1)
