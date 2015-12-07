#
# general_behavior.py -  The primary Owyl behavior tree
# Copyright (C) 2014  Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

# system imports
import copy
import os
import random
import time

# tool imports
import owyl
from owyl import blackboard
import rospy
import roslib
import ConfigParser
import csv

# message imports
from std_msgs.msg import String
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import Target
from blender_api_msgs.msg import BlinkCycle
from blender_api_msgs.msg import SaccadeCycle
from chatbot.msg import ChatMessage
# local stuff.
from face_track import FaceTrack

# Basic holder for emotion-expression properties and probabilities
class Emotion:
    def __init__(self, name) :
        self.name = name
        self.probability = 0.0
        self.min_intensity = 0.0
        self.max_intensity = 1.0
        self.min_duration = 5.0
        self.max_duration = 15.0

# Basic holder for gesture properties and probabilities
class Gesture:
    def __init__(self, name):
        self.name = name
        self.probability = 0.0
        self.min_intensity = 0.0
        self.max_intensity = 1.0
        self.min_repeat = 0.0
        self.max_repeat = 1.0
        self.min_speed = 0.0
        self.max_speed = 1.0

class Tree():
    # ---------
    # Config File utilities
    def unpack_config_emotions(self, config, emo_class) :

        def get_values(from_config, num_values):
            rtn_values = [float(z.strip()) for z in from_config.split(",")]
            if len(rtn_values) != num_values:
                raise Exception("List lengths don't match!")
            return rtn_values

        names = [x.strip() for x in config.get("emotion", emo_class).split(",")]
        numb = len(names)

        probs = get_values(config.get("emotion", \
            emo_class + "_probabilities"), numb)
        mins = get_values(config.get("emotion", \
            emo_class + "_intensity_min"), numb)
        maxs = get_values(config.get("emotion", \
            emo_class + "_intensity_max"), numb)

        dins = get_values(config.get("emotion", \
            emo_class + "_duration_min"), numb)
        daxs = get_values(config.get("emotion", \
            emo_class + "_duration_max"), numb)

        self.blackboard["emotion_classes"].append(emo_class)

        emos = []
        for (n,p,mi,mx,di,dx) in zip (names, probs, mins, maxs, dins, daxs):
            emo = Emotion(n)
            emo.probability = p
            emo.min_intensity = mi
            emo.max_intensity = mx
            emo.min_duration = di
            emo.max_duration = dx
            emos.append(emo)

        self.blackboard[emo_class] = emos

    def unpack_config_gestures(self, config, ges_class) :

        def get_values(from_config, num_values):
            rtn_values = [float(z.strip()) for z in from_config.split(",")]
            if len(rtn_values) != num_values:
                raise Exception("List lengths don't match!")
            return rtn_values

        names = [x.strip() for x in config.get("gesture", ges_class).split(",")]
        numb = len(names)

        probs = get_values(config.get("gesture", \
            ges_class + "_probabilities"), numb)
        mins = get_values(config.get("gesture", \
            ges_class + "_intensity_min"), numb)
        maxs = get_values(config.get("gesture", \
            ges_class + "_intensity_max"), numb)

        rins = get_values(config.get("gesture", \
            ges_class + "_repeat_min"), numb)
        raxs = get_values(config.get("gesture", \
            ges_class + "_repeat_max"), numb)

        sins = get_values(config.get("gesture", \
            ges_class + "_speed_min"), numb)
        saxs = get_values(config.get("gesture", \
            ges_class + "_speed_max"), numb)

        self.blackboard["gesture_classes"].append(ges_class)

        gestures = []
        for (n,p,mi,mx,ri,rx,si,sa) in zip (names, probs, mins, maxs, rins, raxs, sins, saxs):
            ges = Gesture(n)
            ges.probability = p
            ges.min_intensity = mi
            ges.max_intensity = mx
            ges.min_repeat = ri
            ges.max_repeat = rx
            ges.min_speed = si
            ges.max_speed = sa
            gestures.append(ges)

        self.blackboard[ges_class] = gestures

    def unpack_config_look_around(self, config):
        def get_values(from_config, num_values):
            rtn_values = [float(z.strip()) for z in from_config.split(",")]
            if len(rtn_values) != num_values:
                raise Exception("List lengths don't match!")
            return rtn_values

        x_coordinates = [float(x.strip()) for x in config.get("boredom", "search_for_attention_x").split(",")]
        numb = len(x_coordinates)

        y_coordinates = get_values(config.get("boredom", "search_for_attention_y"), numb)
        z_coordinates = get_values(config.get("boredom", "search_for_attention_z"), numb)

        for (x, y, z) in zip (x_coordinates, y_coordinates, z_coordinates):
            trg = Target()
            trg.x = x
            trg.y = y
            trg.z = z
            self.blackboard["search_for_attention_targets"].append(trg)


    def __init__(self):

        self.blackboard = blackboard.Blackboard("rig expressions")

        config = ConfigParser.ConfigParser()
        config.readfp(open(os.path.join(os.path.dirname(__file__), "../behavior.cfg")))
        self.blackboard["current_emotion"] = config.get("emotion", "default_emotion")
        self.blackboard["current_emotion_intensity"] = config.getfloat("emotion", "default_emotion_intensity")
        self.blackboard["current_emotion_duration"] = config.getfloat("emotion", "default_emotion_duration")
        self.blackboard["emotion_classes"] = []
        self.blackboard["gesture_classes"] = []
        self.blackboard["emotion_scale_stage"] = config.getfloat("emotion", "emotion_scale_stage")
        self.blackboard["emotion_scale_closeup"] = config.getfloat("emotion", "emotion_scale_closeup")
        self.blackboard["gesture_scale_stage"] = config.getfloat("gesture", "gesture_scale_stage")
        self.blackboard["gesture_scale_closeup"] = config.getfloat("gesture", "gesture_scale_closeup")

        self.unpack_config_emotions(config, "frustrated_emotions")

        self.unpack_config_emotions(config, "positive_emotions")
        self.unpack_config_emotions(config, "non_positive_emotion")

        self.unpack_config_emotions(config, "bored_emotions")
        self.unpack_config_emotions(config, "non_bored_emotion")

        self.unpack_config_emotions(config, "sleep_emotions")
        self.unpack_config_emotions(config, "non_sleep_emotion")

        self.unpack_config_emotions(config, "wake_up_emotions")

        self.unpack_config_emotions(config, "new_arrival_emotions")

        self.unpack_config_emotions(config, "neutral_speech_emotions")

        self.unpack_config_gestures(config, "positive_gestures")

        self.unpack_config_gestures(config, "bored_gestures")

        self.unpack_config_gestures(config, "sleep_gestures")

        self.unpack_config_gestures(config, "wake_up_gestures")

        self.unpack_config_gestures(config, "listening_gestures")

        self.blackboard["min_duration_for_interaction"] = config.getfloat("interaction", "duration_min")
        self.blackboard["max_duration_for_interaction"] = config.getfloat("interaction", "duration_max")
        self.blackboard["time_to_change_face_target_min"] = config.getfloat("interaction", "time_to_change_face_target_min")
        self.blackboard["time_to_change_face_target_max"] = config.getfloat("interaction", "time_to_change_face_target_max")
        self.blackboard["time_to_change_talking_face_target_min"] = config.getfloat("interaction", "time_to_change_talking_face_target_min")
        self.blackboard["time_to_change_talking_face_target_max"] = config.getfloat("interaction", "time_to_change_talking_face_target_max")
        self.blackboard["glance_probability"] = config.getfloat("interaction", "glance_probability")
        self.blackboard["glance_probability_for_new_faces"] = config.getfloat("interaction", "glance_probability_for_new_faces")
        self.blackboard["glance_probability_for_lost_faces"] = config.getfloat("interaction", "glance_probability_for_lost_faces")
        self.blackboard["z_pitch_eyes"] = config.getfloat("interaction", "z_pitch_eyes")
        self.blackboard["max_glance_distance"] = config.getfloat("interaction", "max_glance_distance")
        self.blackboard["glance_face_duration"] = config.getfloat("interaction", "glance_face_duration")
        self.blackboard["glance_blob_duration"] = config.getfloat("interaction", "glance_blob_duration")
        self.blackboard["min_quick_look_duration"] = config.getfloat("interaction", "min_quick_look_duration")
        self.blackboard["max_quick_look_duration"] = config.getfloat("interaction", "max_quick_look_duration")
        self.blackboard["face_study_probabilities"] = config.getfloat("interaction", "face_study_probabilities")
        self.blackboard["face_study_duration_min"] = config.getfloat("interaction", "face_study_duration_min")
        self.blackboard["face_study_duration_max"] = config.getfloat("interaction", "face_study_duration_max")
        self.blackboard["face_study_z_pitch_nose"] = config.getfloat("interaction", "face_study_z_pitch_nose")
        self.blackboard["face_study_z_pitch_mouth"] = config.getfloat("interaction", "face_study_z_pitch_mouth")
        self.blackboard["face_study_y_pitch_left_ear"] = config.getfloat("interaction", "face_study_y_pitch_left_ear")
        self.blackboard["face_study_y_pitch_right_ear"] = config.getfloat("interaction", "face_study_y_pitch_right_ear")

        # conversational gesture probabilities
        self.blackboard["chatbot_listening_nod_probability"] = config.getfloat("interaction", "chatbot_listening_nod_probability")
        self.blackboard["chatbot_positive_nod_probability"] = config.getfloat("interaction", "chatbot_positive_nod_probability")
        self.blackboard["chatbot_positive_nod_speed_min"] = config.getfloat("interaction", "chatbot_positive_nod_speed_min")
        self.blackboard["chatbot_positive_nod_speed_max"] = config.getfloat("interaction", "chatbot_positive_nod_speed_max")
        self.blackboard["chatbot_positive_nod_magnitude_min"] = config.getfloat("interaction", "chatbot_positive_nod_magnitude_max")
        self.blackboard["chatbot_positive_nod_magnitude_max"] = config.getfloat("interaction", "chatbot_positive_nod_magnitude_max")
        self.blackboard["chatbot_negative_shake_probability"] = config.getfloat("interaction", "chatbot_negative_shake_probability")
        self.blackboard["chatbot_negative_shake_speed_min"] = config.getfloat("interaction", "chatbot_negative_shake_speed_min")
        self.blackboard["chatbot_negative_shake_speed_max"] = config.getfloat("interaction", "chatbot_negative_shake_speed_max")
        self.blackboard["chatbot_negative_shake_magnitude_min"] = config.getfloat("interaction", "chatbot_negative_shake_magnitude_max")
        self.blackboard["chatbot_negative_shake_magnitude_max"] = config.getfloat("interaction", "chatbot_negative_shake_magnitude_max")
        self.blackboard["chatbot_positive_reply_think_probability"] = config.getfloat("interaction", "chatbot_positive_reply_think_probability")
        self.blackboard["chatbot_negative_reply_think_probability"] = config.getfloat("interaction", "chatbot_negative_reply_think_probability")


        self.blackboard["sleep_probability"] = config.getfloat("boredom", "sleep_probability")
        self.blackboard["sleep_duration_min"] = config.getfloat("boredom", "sleep_duration_min")
        self.blackboard["sleep_duration_max"] = config.getfloat("boredom", "sleep_duration_max")
        self.blackboard["search_for_attention_index"] = 0
        self.blackboard["search_for_attention_duration_min"] = config.getfloat("boredom", "search_for_attention_duration_min")
        self.blackboard["search_for_attention_duration_max"] = config.getfloat("boredom", "search_for_attention_duration_max")
        self.blackboard["search_for_attention_targets"] = []

        self.unpack_config_look_around(config)
        self.blackboard["wake_up_probability"] = config.getfloat("boredom", "wake_up_probability")
        self.blackboard["time_to_wake_up"] = config.getfloat("boredom", "time_to_wake_up")
        ### Blinking probablilities
        self.blackboard["chat_heard_probability"] = config.getfloat("blinking", "chat_heard_probability")
        self.blackboard["chat_saying_probability"] = config.getfloat("blinking", "chat_saying_probability")
        self.blackboard["tts_end_probability"] = config.getfloat("blinking", "tts_end_probability")
        self.blackboard["blink_randomly_interval_mean"] = config.getfloat("blinking", "blink_randomly_interval_mean")
        self.blackboard["blink_randomly_interval_var"] = config.getfloat("blinking", "blink_randomly_interval_var")
        self.blackboard["blink_chat_faster_mean"] = config.getfloat("blinking", "blink_chat_faster_mean")
        self.blackboard["blink_chat_slower_mean"] = config.getfloat("blinking", "blink_chat_slower_mean")
        self.blackboard["blink_chat_faster_mean"] = config.getfloat("blinking", "blink_chat_faster_mean")
        ### Saccade probabilities
        self.blackboard["saccade_explore_interval_mean"] = config.getfloat("saccade", "saccade_explore_interval_mean")
        self.blackboard["saccade_explore_interval_var"] = config.getfloat("saccade", "saccade_explore_interval_var")
        self.blackboard["saccade_explore_paint_scale"] = config.getfloat("saccade", "saccade_explore_paint_scale")

        self.blackboard["saccade_study_face_interval_mean"] = config.getfloat("saccade", "saccade_study_face_interval_mean")
        self.blackboard["saccade_study_face_interval_var"] = config.getfloat("saccade", "saccade_study_face_interval_var")
        self.blackboard["saccade_study_face_paint_scale"] = config.getfloat("saccade", "saccade_study_face_paint_scale")

        self.blackboard["saccade_study_face_mouth_width"] = config.getfloat("saccade", "saccade_study_face_mouth_width")
        self.blackboard["saccade_study_face_mouth_height"] = config.getfloat("saccade", "saccade_study_face_mouth_height")
        self.blackboard["saccade_study_face_eye_size"] = config.getfloat("saccade", "saccade_study_face_eye_size")
        self.blackboard["saccade_study_face_eye_distance"] = config.getfloat("saccade", "saccade_study_face_eye_distance")
        self.blackboard["saccade_study_face_weight_eyes"] = config.getfloat("saccade", "saccade_study_face_weight_eyes")
        self.blackboard["saccade_study_face_weight_mouth"] = config.getfloat("saccade", "saccade_study_face_weight_mouth")

        self.blackboard["saccade_micro_interval_mean"] = config.getfloat("saccade", "saccade_micro_interval_mean")
        self.blackboard["saccade_micro_interval_var"] = config.getfloat("saccade", "saccade_micro_interval_var")
        self.blackboard["saccade_micro_paint_scale"] = config.getfloat("saccade", "saccade_micro_paint_scale")

        ##### Other System Variables #####
        self.blackboard["show_expression_since"] = None

        # ID's of faces newly seen, or lost. Integer ID.
        self.blackboard["new_face"] = 0
        self.blackboard["lost_face"] = 0
        # IDs of faces in the scene, updated once per cycle
        self.blackboard["face_targets"] = []
        self.blackboard["blob_targets"] = []
        self.blackboard["talking_faces"] = []
        self.blackboard["recognized_face_targets"] = []
        # IDs of faces in the scene, updated immediately
        self.blackboard["background_face_targets"] = []
        self.blackboard["background_blob_targets"] = []
        self.blackboard["background_talking_faces"] = []
        self.blackboard["background_recognized_face_targets"] = []
        self.blackboard["background_x_recognized_face_targets"] = []
        self.blackboard["current_glance_target"] = 0
        self.blackboard["current_face_target"] = 0
        self.blackboard["new_look_at_face"] = 0
        self.blackboard["rs_face_targets"] = []
        # A recognized face and its name
        self.blackboard["recog_face"] = 0
        self.blackboard["recog_face_name"] = ""
        self.blackboard["interact_with_face_target_since"] = 0.0
        self.blackboard["sleep_since"] = 0.0
        self.blackboard["bored_since"] = 0.0
        self.blackboard["is_interruption"] = False
        self.blackboard["is_sleeping"] = False
        self.blackboard["behavior_tree_on"] = False
        self.blackboard["stage_mode"] = False
        # Flags to indicate which part of the face will be studied
        self.blackboard["face_study_nose"] = False
        self.blackboard["face_study_mouth"] = False
        self.blackboard["face_study_left_ear"] = False
        self.blackboard["face_study_right_ear"] = False

        # flag to indicate in conversation and use different emotion set and weights
        self.conversing = False

        ##### ROS Connections #####
        self.facetrack = FaceTrack(self.blackboard)

        rospy.Subscriber("/behavior_switch", String, self.behavior_switch_callback)
        rospy.Subscriber("/blender_api/available_emotion_states",
            AvailableEmotionStates, self.get_emotion_states_cb)

        rospy.Subscriber("/blender_api/available_gestures",
            AvailableGestures, self.get_gestures_cb)

        # Emotional content that the chatbot perceived i.e. did it hear
        # (or reply with) angry words, polite words, etc?
        # Currently chatbot supplies string rather than specific EmotionState with timing,
        # allowing that to be handled here where timings have been tuned
        rospy.logwarn("setting up chatbot affect perceive and express links")
        rospy.Subscriber("chatbot_affect_perceive", String,
            self.chatbot_affect_perceive_callback)

        #  Handle messages from incoming speech to simulate listening engagement
        rospy.Subscriber("chat_events",String,self.chat_event_dispatcher_callback)

        #  chatbot can request blinks correlated with hearing and speaking
        rospy.Subscriber("chatbot_blink",String,self.chatbot_blink_callback)



        self.emotion_pub = rospy.Publisher("/blender_api/set_emotion_state",
            EmotionState, queue_size=1)
        self.gesture_pub = rospy.Publisher("/blender_api/set_gesture",
            SetGesture, queue_size=1)
        self.affect_pub = rospy.Publisher("chatbot_affect_express",
            EmotionState, queue_size=1)
        self.blink_pub = rospy.Publisher("/blender_api/set_blink_randomly",
            BlinkCycle,queue_size=1)
        self.saccade_pub = rospy.Publisher("/blender_api/set_saccade",
            SaccadeCycle,queue_size=1)
        self.chat_pub = rospy.Publisher("/han/chatbot_responses", String, queue_size=1)

        self.do_pub_gestures = True
        self.do_pub_emotions = True

        self.tree = self.build_tree()
        time.sleep(0.1)

        log_filename = os.path.join(os.path.abspath(
                os.path.dirname(__file__)), "../bhlog.csv")
        self.log_file = open(log_filename, 'wb')
        self.log_file.write('Action,Timestamp,Event\n')
        try:
            while not rospy.is_shutdown():
                self.tree.next()
        finally:
            self.log_file.close()


    # Pick a random expression out of the class of expressions,
    # and display it. Return the display emotion, or None if none
    # were picked.  Optional argument force guarantees a return.
    def pick_random_expression(self, emo_class_name, trigger, force=False):
        random_number = random.random()
        tot = 0
        emo = None
        emos = self.blackboard[emo_class_name]
        # print emos
        for emotion in emos:
            tot += emotion.probability
            if random_number <= tot:
                emo = emotion
                break
        if emo:
            if force==True:
                # may want to make this variable in .cfg
                intensity = random.uniform(emo.min_intensity, emo.max_intensity)
                duration = emo.min_duration
            else:
                intensity = random.uniform(emo.min_intensity, emo.max_intensity)
                duration = random.uniform(emo.min_duration, emo.max_duration)
            self.show_emotion(emo.name, intensity, duration, trigger)
        elif force==True:
            # force said show something but nothing picked, so choose first
            print 'force branch chosen'
            emo=emos[0]
            intensity = emo.max_intensity
            duration = emo.max_duration
            self.show_emotion(emo.name, intensity, duration, trigger)

        return emo

    def pick_random_gesture(self, ges_class_name, trigger):
        random_number = random.random()
        tot = 0
        ges = None
        gestures = self.blackboard[ges_class_name]
        for gesture in gestures:
            tot += gesture.probability
            if random_number <= tot:
                ges = gesture
                break

        if ges:
            intensity = random.uniform(ges.min_intensity, ges.max_intensity)
            repeat = random.uniform(ges.min_repeat, ges.max_repeat)
            speed = random.uniform(ges.min_speed, ges.max_speed)
            self.show_gesture(ges.name, intensity, repeat, speed, trigger)

        return ges


    # Pick the name of a random emotion, excluding those from
    # the exclude list
    def pick_random_emotion_name(self, exclude) :
        ixnay = [ex.name for ex in exclude]
        emos = self.blackboard["emotions"]
        if None == emos:
            return None
        emo_name = random.choice([other for other in emos if other not in ixnay])
        return emo_name

    # Pick a  so-called "instant" or "flash" expression
    def pick_instant(self, emo_class, exclude_class, trigger) :
        emo = self.pick_random_expression(exclude_class)
        if emo :
            exclude = self.blackboard[emo_class]
            emo_name = self.pick_random_emotion_name(exclude)
            tense = random.uniform(emo.min_intensity, emo.max_intensity)
            durat = random.uniform(emo.min_duration, emo.max_duration)
            self.show_emotion(emo_name, tense, durat, trigger)
            print "----- Instant expression: " + emo_name + " (" + \
                 str(tense) + ") for " + str(durat) + " seconds"
        return emo_name

    # ------------------------------------------------------------------
    # The various behavior trees

    # Actions that are taken when a face becomes visible.
    # If there were no people in the scene, she always interacts with that person
    # If she is already interacting with someone else in the scene,
    # she will either glance at the new face or ignore it, depends on the dice roll
    # If she has been interacting with another person for a while,
    # the probability of glancing at a new face is higher
    def someone_arrived(self) :
        tree = owyl.sequence(
            self.is_someone_arrived(),
            owyl.selector(
                ##### There previously were no people in the scene #####
                owyl.sequence(
                    self.were_no_people_in_the_scene(),
                    self.assign_face_target(variable="current_face_target", value="new_face"),
                    self.record_start_time(variable="interact_with_face_target_since"),
                    self.show_expression(emo_class="new_arrival_emotions", trigger="someone_arrived"),
                    self.interact_with_face_target(id="current_face_target", new_face=True, trigger="someone_arrived")
                ),

                ##### Currently interacting with someone #####
                owyl.sequence(
                    self.is_interacting_with_someone(),
                    self.dice_roll(event="glance_new_face"),
                    self.glance_at_new_face(trigger="someone_arrived")
                ),

                ##### Does Nothing #####
                owyl.sequence(
                    self.print_status(str="----- Ignoring the new face!"),
                    self.log(behavior="ignore_face", trigger="someone_arrived"),
                    owyl.succeed()
                )
            ),
            self.clear_new_face_target()
        )
        return tree

    # ---------------------------
    # Actions that are taken when a face leaves
    # If she was interacting with that person, she will be frustrated
    # If she was interacting with someone else,
    # she will either glance at the lost face or ignore it, depends on the dice roll
    def someone_left(self) :
        tree = owyl.sequence(
            self.is_someone_left(),
            owyl.selector(
                ##### Was Interacting With That Person #####
                owyl.sequence(
                    self.was_interacting_with_that_person(),
                    self.return_to_neutral_position(trigger="someone_left"),
                    self.show_frustrated_expression(trigger="someone_left")
                ),

                ##### Is Interacting With Someone Else #####
                owyl.sequence(
                    self.is_interacting_with_someone(),
                    self.dice_roll(event="glance_lost_face"),
                    self.glance_at_lost_face(trigger="someone_left")
                ),

                ##### Does Nothing #####
                owyl.sequence(
                    self.print_status(str="----- Ignoring the lost face!"),
                    self.log(behavior="ignore_face", trigger="someone_left"),
                    owyl.succeed()
                )
            ),
            self.clear_lost_face_target()
        )
        return tree

    # -----------------------------
    # Interact with people who are talking
    # If someone is talking and she is not currently interacting with any of them, interact with one of them
    # If it is time to switch target, interact with someone else who is talking
    # If only one person is talking, keep interacting with that person
    # Otherwise she will continue with the current interaction
    # she may also quickly look at other people if there are more than one people in the scene
    def interact_with_talking_people(self):
        tree = owyl.sequence(
            self.is_someone_talking(),
            owyl.selector(
                ##### Interact With A Talking Person #####
                owyl.sequence(
                    owyl.selector(
                        self.is_not_interacting_with_a_talking_person(),
                        self.is_only_one_person_talking()
                    ),
                    self.select_a_talking_face_target(),
                    self.record_start_time(variable="interact_with_face_target_since"),
                    self.interact_with_face_target(id="current_face_target", new_face=False, trigger="someone_is_talking"),
                    self.print_status(str="----- Interact with a talking person")
                ),

                ##### Start A New Interaction With A Talking Person #####
                owyl.sequence(
                    owyl.selector(
                        self.is_not_interacting_with_someone(),
                        owyl.sequence(
                            self.is_more_than_one_person_talking(),
                            self.is_time_to_change_face_target(min="time_to_change_talking_face_target_min", max="time_to_change_talking_face_target_max")
                        )
                    ),
                    self.select_a_talking_face_target(),
                    self.record_start_time(variable="interact_with_face_target_since"),
                    self.interact_with_face_target(id="current_face_target", new_face=False, trigger="someone_is_talking"),
                    self.print_status(str="----- Started a new interaction with another talking person")
                ),

                ##### Quick-look At Other Faces & Continue With The Last Interaction #####
                owyl.sequence(
                    owyl.selector(
                        owyl.sequence(
                            self.is_more_than_one_face_target(),
                            # TODO: Maybe to define a new config for the someone is talking case
                            self.dice_roll(event="group_interaction"),
                            self.select_a_quick_look_target(),
                            self.quick_look_at(id="current_quick_look_target", trigger="someone_is_talking")
                        ),
                        owyl.succeed()
                    ),
                    self.interact_with_face_target(id="current_face_target", new_face=False, trigger="someone_is_talking"),
                    self.print_status(str="----- Continue interacting with a talking person")
                )
            )
        )
        return tree

    # -----------------------------
    def interact_with_recognized_people(self):
        tree = owyl.sequence(
            self.is_a_recognized_face_to_be_greeted(),
            self.print_status(str="----- Greet a recognized face"),
            self.assign_face_target(variable="current_face_target", value="recog_face"),
            self.record_start_time(variable="interact_with_face_target_since"),
            self.interact_with_face_target(id="current_face_target", new_face=False, trigger="recognized_someone"),
            self.greet(id="current_face_target", name="recog_face_name", trigger="recognized_someone"),
            self.clear_recognized_face()
        )
        return tree

    # -----------------------------
    # Interact with people
    # If she is not currently interacting with anyone, or it's time to switch target
    # she will start interacting with someone else
    # Otherwise she will continue with the current interaction
    # she may also glance at other people if there are more than one people in the scene
    def interact_with_people(self) :
        tree = owyl.sequence(
            self.is_face_target(),
            owyl.selector(
                ##### Start A New Interaction #####
                owyl.sequence(
                    owyl.selector(
                        self.is_not_interacting_with_someone(),
                        owyl.sequence(
                            self.is_more_than_one_face_target(),
                            self.is_time_to_change_face_target(min="time_to_change_face_target_min", max="time_to_change_face_target_max")
                        )
                    ),
                    self.select_a_face_target(),
                    self.record_start_time(variable="interact_with_face_target_since"),
                    self.interact_with_face_target(id="current_face_target", new_face=False, trigger="people_in_scene"),
                    self.print_status(str="----- Started a new interaction")
                ),

                ##### Glance At Other Faces & Continue With The Last Interaction #####
                owyl.sequence(
                    self.print_status(str="----- Continue interaction"),
                    owyl.selector(
                        owyl.sequence(
                            self.is_more_than_one_face_target(),
                            self.dice_roll(event="group_interaction"),
                            self.select_a_glance_target(),
                            self.glance_at(id="current_glance_target", trigger="people_in_scene")
                        ),
                        owyl.succeed()
                    ),
                    self.interact_with_face_target(id="current_face_target", new_face=False, trigger="people_in_scene"),
                    owyl.selector(
                        owyl.sequence(
                            self.dice_roll(event="face_study_saccade"),
                            self.face_study_saccade(id="current_face_target", trigger="people_in_scene")
                        ),
                        owyl.succeed()
                    )
                )
            )
        )
        return tree

    # -------------------
    # Nothing interesting is happening.
    # She will look around and search for attention.  She may go to sleep,
    # and it's more likely to happen if she has been bored for a while.
    # She wakes up whenever there's an interruption, e.g. someone arrives
    # or after a timeout.
    def nothing_is_happening(self) :
        tree = owyl.sequence(
            owyl.selector(
                ##### Is Not Sleeping #####
                owyl.sequence(
                    self.is_not_sleeping(),
                    owyl.selector(
                        ##### Go To Sleep #####
                        owyl.sequence(
                            self.dice_roll(event="go_to_sleep"),
                            self.record_start_time(variable="sleep_since"),
                            self.print_status(str="----- Go to sleep!"),
                            self.go_to_sleep(trigger="nothing_is_happening")
                        ),

                        ##### Search For Attention #####
                        self.search_for_attention(trigger="nothing_is_happening")
                    )
                ),

                ##### Is Sleeping #####
                owyl.selector(
                    ##### Wake Up #####
                    owyl.sequence(
                        self.dice_roll(event="wake_up"),
                        self.is_time_to_wake_up(),
                        self.wake_up(trigger="time_to_wake_up"),
                    ),

                    ##### Continue To Sleep #####
                    owyl.sequence(
                        self.print_status(str="----- Continue to sleep."),
                        self.go_to_sleep(trigger="nothing_is_happening")
                    )
                )
            ),

            ##### If Interruption && Sleeping -> Wake Up #####
            owyl.sequence(
                self.is_interruption(),
                self.is_sleeping(),
                self.wake_up(trigger="interruption"),
                self.print_status(str="----- Interruption: Wake up!"),
            )
        )
        return tree

    # ------------------------------------------------------------------
    # If operator sets face it should change current face target
    def face_set_by_operator(self):
        tree = owyl.sequence(
            self.is_someone_selected(),
            self.is_not_current_face(id="new_look_at_face"),
            self.assign_face_target(variable="current_face_target", value="new_look_at_face"),
            self.assign_var_value(variable="new_look_at_face", value=0),
            self.record_start_time(variable="interact_with_face_target_since"),
            self.show_expression(emo_class="new_arrival_emotions", trigger="new_person_selected"),
            self.interact_with_face_target(id="current_face_target", new_face=True, trigger="new_person_selected")
        )
        return tree

    # ------------------------------------------------------------------
    # Build the main tree
    def build_tree(self):
        eva_behavior_tree = \
            owyl.repeatAlways(
                owyl.selector(
                    owyl.sequence(
                        self.is_behavior_tree_on(),
                        self.sync_variables(),
                        ########## Main Events ##########
                        owyl.selector(
                            self.face_set_by_operator(),
                            self.someone_arrived(),
                            self.someone_left(),
                            self.interact_with_talking_people(),
                            self.interact_with_recognized_people(),
                            self.interact_with_people(),
                            self.nothing_is_happening()
                        )
                    ),
                    self.idle_spin()
                )
            )
        return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

    # Print a single status message
    @owyl.taskmethod
    def print_status(self, **kwargs):
        print kwargs["str"]
        yield True

    @owyl.taskmethod
    def assign_var_value(self, **kwargs):
        self.blackboard[kwargs["variable"]] = kwargs["value"]
        yield True

    # Print emotional state
    @owyl.taskmethod
    def sync_variables(self, **kwargs):
        self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
        self.blackboard["talking_faces"] = self.blackboard["background_talking_faces"]
        self.blackboard["blob_targets"] = self.blackboard["background_blob_targets"]
        self.blackboard["recognized_face_targets"] = self.blackboard["background_x_recognized_face_targets"]
        print "Visible faces: ", self.blackboard["face_targets"]
        print "Talking faces: ", self.blackboard["talking_faces"]
        yield True

    @owyl.taskmethod
    def dice_roll(self, **kwargs):
        if kwargs["event"] == "glance_new_face":
            if self.blackboard["glance_probability_for_new_faces"] > 0 and self.blackboard["interact_with_face_target_since"] > 0:
                skew = (time.time() - self.blackboard["interact_with_face_target_since"]) / self.blackboard["time_to_change_face_target_max"]
                if random.random() < self.blackboard["glance_probability_for_new_faces"] + skew:
                    yield True
                else:
                    yield False
            else:
                yield False
        elif kwargs["event"] == "group_interaction":
            if random.random() < self.blackboard["glance_probability"]:
                yield True
            else:
                yield False
        elif kwargs["event"] == "face_study_saccade":
            if random.random() < self.blackboard["face_study_probabilities"]:
                yield True
            else:
                yield False
        elif kwargs["event"] == "go_to_sleep":
            if self.blackboard["sleep_probability"] > 0 and self.blackboard["bored_since"] > 0:
                skew = (time.time() - self.blackboard["bored_since"]) / \
                       (self.blackboard["search_for_attention_duration_max"] / self.blackboard["sleep_probability"])
                if random.random() < self.blackboard["sleep_probability"] + skew:
                    yield True
                else:
                    yield False
            else:
                yield False
        elif kwargs["event"] == "wake_up":
            if random.random() < self.blackboard["wake_up_probability"]:
                yield True
            else:
                yield False
        else:
            if random.random() > 0.5:
                yield True
            else:
                yield False

    @owyl.taskmethod
    def is_someone_arrived(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["new_face"] > 0:
            self.blackboard["bored_since"] = 0
            print("----- Someone arrived! id: " + str(self.blackboard["new_face"]))
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_selected(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["new_look_at_face"] > 0:
            self.blackboard["bored_since"] = 0
            print("----- Someone selected! id: " + str(self.blackboard["new_look_at_face"]))
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_a_recognized_face_to_be_greeted(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["recog_face"] > 0:
            self.blackboard["bored_since"] = 0
            print("----- Recognized someone! id: " + str(self.blackboard["recog_face"]))
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_not_current_face(self, **kwargs):
        if self.blackboard["current_face_target"] == self.blackboard[kwargs["id"]]:
            print("----- Is Interacting with id: {} already".format(self.blackboard[kwargs["id"]]))
            yield False
        else:
            yield True

    @owyl.taskmethod
    def is_someone_left(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["lost_face"] > 0:
            print("----- Someone left! id: " + str(self.blackboard["lost_face"]))
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_interacting_with_someone(self, **kwargs):
        if self.blackboard["current_face_target"]:
            "----- Is Interacting With Someone!"
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_not_interacting_with_someone(self, **kwargs):
        if not self.blackboard["current_face_target"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_not_interacting_with_a_talking_person(self, **kwargs):
        if not self.blackboard["current_face_target"] in self.blackboard["talking_faces"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def were_no_people_in_the_scene(self, **kwargs):
        if len(self.blackboard["face_targets"]) == 1:
            print("----- Previously, no one in the scene!")
            yield True
        else:
            yield False

    @owyl.taskmethod
    def was_interacting_with_that_person(self, **kwargs):
        if self.blackboard["current_face_target"] == self.blackboard["lost_face"]:
            self.blackboard["current_face_target"] = 0
            print("----- Lost face " + str(self.blackboard["lost_face"]) +
                ", but was interacting with them!")
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_face_target(self, **kwargs):
        if len(self.blackboard["face_targets"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_talking(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if len(self.blackboard["talking_faces"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_no_one_talking(self, **kwargs):
        if len(self.blackboard["talking_faces"]) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_more_than_one_person_talking(self, **kwargs):
        if len(self.blackboard["talking_faces"]) > 1:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_more_than_one_face_target(self, **kwargs):
        if len(self.blackboard["face_targets"]) > 1:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_only_one_person_talking(self, **kwargs):
        if len(self.blackboard["talking_faces"]) == 1:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_time_to_change_face_target(self, **kwargs):
        if self.blackboard["interact_with_face_target_since"] > 0 and \
                (time.time() - self.blackboard["interact_with_face_target_since"]) >= \
                        random.uniform(self.blackboard[kwargs["min"]], self.blackboard[kwargs["max"]]):
            print "----- Time to start a new interaction!"
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_time_to_wake_up(self, **kwargs):
        if self.blackboard["sleep_since"] > 0 and (time.time() - self.blackboard["sleep_since"]) >= self.blackboard["time_to_wake_up"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_sleeping(self, **kwargs):
        if self.blackboard["is_sleeping"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_not_sleeping(self, **kwargs):
        if not self.blackboard["is_sleeping"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_interruption(self, **kwargs):
        if self.blackboard["is_interruption"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_behavior_tree_on(self, **kwargs):
        if self.blackboard["behavior_tree_on"]:
            # Print an empty line, it is clearer to see the print_status msgs in each cycle
            print ""
            yield True
        else:
            yield False

    @owyl.taskmethod
    def assign_face_target(self, **kwargs):
        self.blackboard[kwargs["variable"]] = self.blackboard[kwargs["value"]]
        yield True

    @owyl.taskmethod
    def select_a_talking_face_target(self, **kwargs):
        self.blackboard["current_face_target"] = FaceTrack.random_face_target(self.blackboard["talking_faces"], self.blackboard["current_face_target"])
        yield True

    @owyl.taskmethod
    def select_a_face_target(self, **kwargs):
        # Select a recognized face, if any
        if self.blackboard["recognized_face_targets"]:
            self.blackboard["current_face_target"] = FaceTrack.random_face_target(self.blackboard["recognized_face_targets"])
        else:
            self.blackboard["current_face_target"] = FaceTrack.random_face_target(self.blackboard["face_targets"])
        yield True

    @owyl.taskmethod
    def select_a_glance_target(self, **kwargs):
        self.blackboard["current_glance_target"] = FaceTrack.random_face_target(self.blackboard["face_targets"], self.blackboard["current_face_target"])
        yield True

    @owyl.taskmethod
    def select_a_quick_look_target(self, **kwargs):
        self.blackboard["current_quick_look_target"] = FaceTrack.random_face_target(self.blackboard["face_targets"], self.blackboard["current_face_target"])
        yield True

    @owyl.taskmethod
    def record_start_time(self, **kwargs):
        self.blackboard[kwargs["variable"]] = time.time()
        yield True

    @owyl.taskmethod
    def quick_look_at(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        trigger = kwargs["trigger"]
        self.facetrack.look_at_face(face_id)
        self.write_log("quick_look_at_" + str(face_id), time.time(), trigger)

        interval = 0.01
        duration = random.uniform(self.blackboard["min_quick_look_duration"], self.blackboard["max_quick_look_duration"])
        self.break_if_interruptions(interval, duration)
        yield True

    @owyl.taskmethod
    def interact_with_face_target(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        trigger = kwargs["trigger"]
        self.facetrack.look_at_face(face_id)
        self.write_log("look_at_" + str(face_id), time.time(), trigger)

        if self.should_show_expression("positive_emotions") or kwargs["new_face"]:
            if self.conversing:
                self.pick_random_expression("neutral_speech_emotions",trigger)
            else:
                if random.random() < self.blackboard["non_positive_emotion_probabilities"]:
                    self.pick_instant("positive_emotions", "non_positive_emotion", trigger)
                else:
                    self.pick_random_expression("positive_emotions", trigger)
        ##### Show A Positive Gesture #####
        self.pick_random_gesture("positive_gestures", trigger)
        ##### if new face, show affiliative brow raise  #####
        if trigger=="someone_arrived":
          if random.random()<0.8:
              # TODO add probability, speed,intensity to config file and read
              speed=random.uniform(0.2,0.6)
              intensity=random.uniform(0.4,0.7)
              # TODO use non-blocking sleep from GM
              time.sleep(0.5)
              self.show_gesture("think-browsUp",intensity,1,speed,trigger)

        interval = 0.01
        duration = random.uniform(self.blackboard["min_duration_for_interaction"], self.blackboard["max_duration_for_interaction"])
        print "----- Interacting w/face id:" + str(face_id) + " for " + str(duration)[:5] + " seconds"
        self.break_if_interruptions(interval, duration)
        yield True

    @owyl.taskmethod
    def greet(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        name = self.blackboard[kwargs["name"]]
        trigger = kwargs["trigger"]

        self.write_log("greeting: " + str(face_id), time.time(), trigger)

        msg = "Hi " + name
        self.chat_pub.publish(msg)

    @owyl.taskmethod
    def face_study_saccade(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        duration = random.uniform(self.blackboard["face_study_duration_min"], self.blackboard["face_study_duration_max"])

        # Randomly pick which part of the face to study
        which_part = random.randint(1, 4)
        if which_part == 1:
            self.blackboard["face_study_nose"] = True
            print "----- Studying face:" + str(face_id) + " (nose)"
        elif which_part == 2:
            self.blackboard["face_study_mouth"] = True
            print "----- Studying face:" + str(face_id) + " (mouth)"
        elif which_part == 3:
            self.blackboard["face_study_left_ear"] = True
            print "----- Studying face:" + str(face_id) + " (left ear)"
        elif which_part == 4:
            self.blackboard["face_study_right_ear"] = True
            print "----- Studying face:" + str(face_id) + " (right ear)"

        self.facetrack.study_face(face_id, duration)
        self.write_log("face_study", time.time(), kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def glance_at(self, **kwargs):
        target_id = self.blackboard[kwargs["id"]]
        print "----- Glancing at face/blob:" + str(target_id)
        if target_id in self.blackboard["face_targets"]:
            self.facetrack.glance_at_face(target_id, self.blackboard["glance_face_duration"])
        else:
            self.facetrack.glance_at_face(target_id, self.blackboard["glance_blob_duration"])
        self.write_log("glance_at_" + str(target_id), time.time(), kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def glance_at_new_face(self, **kwargs):
        face_id = self.blackboard["new_face"]
        print "----- Glancing at new face:" + str(face_id)
        self.facetrack.glance_at_face(face_id, self.blackboard["glance_face_duration"])
        self.write_log("glance_at_" + str(face_id), time.time(), kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def glance_at_lost_face(self, **kwargs):
        print "----- Glancing at lost face:" + str(self.blackboard["lost_face"])
        face_id = self.blackboard["lost_face"]
        self.facetrack.glance_at_face(face_id, 1)
        self.write_log("glance_at_" + str(face_id), time.time(), kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def show_expression(self, **kwargs):
        self.pick_random_expression(kwargs["emo_class"], kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def show_frustrated_expression(self, **kwargs):
        self.pick_random_expression("frustrated_emotions", kwargs["trigger"])
        yield True

    @owyl.taskmethod
    def return_to_neutral_position(self, **kwargs):
        self.facetrack.look_at_face(0)
        self.write_log("look_at_neutral", time.time(), kwargs["trigger"])
        yield True

    # Accept an expression name, intensity and duration, and publish it
    # as a ROS message both to blender, and to the chatbot.  Currently,
    # exactly the same message format is used for both blender and the
    # chatbot. This may change in the future(?)
    def show_emotion(self, expression, intensity, duration, trigger):
        # Try to avoid showing more than one expression at once
        now = time.time()
        since = self.blackboard["show_expression_since"]
        durat = self.blackboard["current_emotion_duration"]
        if since is not None and (now - since < 0.7 * durat) :
        # chat triggers will override random probabilistic emotion choices
                if trigger !='chat_perceived':
                    return

        # Update the blackboard
        self.blackboard["current_emotion"] = expression
        self.blackboard["current_emotion_intensity"] = intensity
        self.blackboard["current_emotion_duration"] = duration

        # Create the message
        exp = EmotionState()
        exp.name = self.blackboard["current_emotion"]
        exp.magnitude = self.blackboard["current_emotion_intensity"]
        intsecs = int(duration)
        exp.duration.secs = intsecs
        exp.duration.nsecs = 1000000000 * (duration - intsecs)
        # emotion_pub goes to blender and tts;
        if (self.do_pub_emotions) or trigger=='chat_perceived' :
            self.emotion_pub.publish(exp)
            self.write_log(exp.name, time.time(), trigger)

            print "----- Show expression: " + expression + " (" + str(intensity)[:5] + ") for " + str(duration)[:4] + " seconds"
            self.blackboard["show_expression_since"] = time.time()

    # Accept an gesture name, intensity, repeat (perform how many times)
    # and speed and then publish it as a ros message.
    # chat triggers may override general behavior setting
    # this may be generalized to an admissible trigger set
    def show_gesture(self, gesture, intensity, repeat, speed, trigger):
        ges = SetGesture()
        ges.name = gesture
        ges.magnitude = intensity
        ges.repeat = repeat
        ges.speed = speed
        if (self.do_pub_gestures) or trigger=='chat_perceived':
            self.gesture_pub.publish(ges)
            self.write_log(ges.name, time.time(), trigger)


        print "----- Show gesture: " + gesture + " (" + str(intensity)[:5] + ")"

    @owyl.taskmethod
    def search_for_attention(self, **kwargs):
        print("----- Search for attention")
        trigger = kwargs["trigger"]
        if self.blackboard["bored_since"] == 0:
            self.blackboard["bored_since"] = time.time()

        # Send out the look around msg
        current_idx = self.blackboard["search_for_attention_index"]
        look_around_trg = self.blackboard["search_for_attention_targets"][current_idx]
        self.facetrack.look_pub.publish(look_around_trg)
        self.write_log("look_around", time.time(), trigger)

        # Update / reset the index
        if self.blackboard["search_for_attention_index"] + 1 < len(self.blackboard["search_for_attention_targets"]):
            self.blackboard["search_for_attention_index"] += 1
        else:
            self.blackboard["search_for_attention_index"] = 0

        if self.should_show_expression("bored_emotions"):
            # Show a bored expression, either with or without an instant expression in advance
            if random.random() < self.blackboard["non_bored_emotion_probabilities"]:
                self.pick_instant("bored_emotions", "non_bored_emotion", trigger)
            else:
                self.pick_random_expression("bored_emotions", trigger)

        ##### Show A Bored Gesture #####
        self.pick_random_gesture("bored_gestures", trigger)

        interval = 0.01
        duration = random.uniform(self.blackboard["search_for_attention_duration_min"], self.blackboard["search_for_attention_duration_max"])
        self.break_if_interruptions(interval, duration)
        yield True

    # To determine whether it is a good time to show another expression
    # Can be used to avoid making expressions too frequently
    def should_show_expression(self, emo_class):
        since = self.blackboard["show_expression_since"]
        if since is not None and (time.time() - since) >= (self.blackboard["current_emotion_duration"] / 4):
            return True
        else:
            return False

    @owyl.taskmethod
    def go_to_sleep(self, **kwargs):
        self.blackboard["is_sleeping"] = True
        self.blackboard["bored_since"] = 0.0

        ##### Show A Sleep Expression #####
        self.pick_random_emotion_name(self.blackboard["sleep_emotions"])

        ##### Show A Sleep Gesture #####
        self.pick_random_gesture("sleep_gestures", "nothing_is_happening")

        interval = 0.01
        duration = random.uniform(self.blackboard["sleep_duration_min"], self.blackboard["sleep_duration_max"])
        self.break_if_interruptions(interval, duration)
        yield True

    @owyl.taskmethod
    def wake_up(self, **kwargs):
        print "----- Wake up!"
        trigger = kwargs["trigger"]
        self.blackboard["is_sleeping"] = False
        self.blackboard["sleep_since"] = 0.0
        self.blackboard["bored_since"] = 0.0

        ##### Show A Wake Up Expression #####
        self.pick_random_expression("wake_up_emotions", trigger)

        ##### Show A Wake Up Gesture #####
        self.pick_random_gesture("wake_up_gestures", trigger)

        yield True

    @owyl.taskmethod
    def clear_new_face_target(self, **kwargs):
        #if not self.blackboard["is_interruption"]:
        self.blackboard["new_face"] = 0
        yield True

    @owyl.taskmethod
    def clear_lost_face_target(self, **kwargs):
        self.blackboard["lost_face"] = 0
        yield True

    @owyl.taskmethod
    def clear_recognized_face(self, **kwargs):
        self.blackboard["recog_face"] = 0
        self.blackboard["recog_face_name"] = ""
        yield True

    # This avoids burning CPU time when the behavior system is off.
    # Mostly it sleeps, and periodically checks for interrpt messages.
    @owyl.taskmethod
    def idle_spin(self, **kwargs):
        if self.blackboard["behavior_tree_on"]:
            yield True

        # Sleep for 1 second.
        time.sleep(1)
        yield True

    def break_if_interruptions(self, interval, duration):
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break

    # Return the subset of 'core' strings that are in 'avail' strings.
    # Note that 'avail' strings might contain longer names,
    # e.g. "happy-3", whereas core just contains "happy". We want to
    # return "happy-3" in that case, as well as happy-2 and happy-1
    # if they are there.
    def set_intersect(self, emo_class, avail) :
        emos = self.blackboard[emo_class]
        rev = []
        for emo in emos:
            for a in avail:
                if emo.name in a:
                    # Copy the emotion, but give it the new name!
                    nemo = copy.deepcopy(emo)
                    nemo.name = a
                    rev.append(nemo)

        # Now, renormalize the probabilities
        tot = 0.0
        for emo  in rev:
            tot += emo.probability
        for emo  in rev:
            emo.probability /= tot

        self.blackboard[emo_class] = rev

    @owyl.taskmethod
    def log(self, **kwargs):
        self.write_log(kwargs["behavior"], time.time(), kwargs["trigger"])

    def write_log(self, behavior, log_time, trigger):
        logger = csv.writer(
            self.log_file, delimiter=",", lineterminator='\n')
        logger.writerow([behavior, log_time, trigger])
        self.log_file.flush()

    # Get the list of available emotions. Update our master list,
    # and cull the various subclasses appropriately.
    def get_emotion_states_cb(self, msg) :
        print("Available Emotion States:" + str(msg.data))
        # Update the complete list of emtions.
        self.blackboard["emotions"] = msg.data

        # Reconcile the other classes
        self.set_intersect("frustrated_emotions", msg.data)
        self.set_intersect("positive_emotions", msg.data)
        self.set_intersect("bored_emotions", msg.data)
        self.set_intersect("sleep_emotions", msg.data)
        self.set_intersect("wake_up_emotions", msg.data)
        self.set_intersect("new_arrival_emotions", msg.data)


    def get_gestures_cb(self, msg) :
        print("Available Gestures:" + str(msg.data))

    # Rescale the intensity of the expressions.
    def rescale_intensity(self, emo_scale, gest_scale) :
        for emo_class in self.blackboard["emotion_classes"]:
            for emo in self.blackboard[emo_class]:
                emo.min_intensity *= emo_scale
                emo.max_intensity *= emo_scale

        for ges_class in self.blackboard["gesture_classes"]:
            for ges in self.blackboard[ges_class]:
                ges.min_intensity *= gest_scale
                ges.max_intensity *= gest_scale

    # Turn behaviors on and off.
    def behavior_switch_callback(self, data):
        if data.data == "btree_on":
            self.do_pub_gestures = True
            self.do_pub_emotions = True
            self.blackboard["is_interruption"] = False

            # initialize behavior linked procedural cycle defaults
            self.init_blink()
            self.init_saccade()

            emo_scale = self.blackboard["emotion_scale_closeup"]
            ges_scale = self.blackboard["gesture_scale_closeup"]

            # If the current mode is stage mode, then tone things down.
            if self.blackboard["stage_mode"]:
                print("----- Switch to close-up mode")
                emo_scale /= self.blackboard["emotion_scale_stage"]
                ges_scale /= self.blackboard["gesture_scale_stage"]

            else:
                print("----- Behavior tree enabled, closeup mode.")

            self.rescale_intensity(emo_scale, ges_scale)
            self.blackboard["stage_mode"] = False
            self.blackboard["behavior_tree_on"] = True

        elif data.data == "btree_on_stage":
            self.do_pub_gestures = True
            self.do_pub_emotions = True
            self.blackboard["is_interruption"] = False

            # initialize behavior linked procedural cycle defaults
            self.init_blink()
            self.init_saccade()


            emo_scale = self.blackboard["emotion_scale_stage"]
            ges_scale = self.blackboard["gesture_scale_stage"]

            # If previously in close-up mode, exaggerate the emotions
            # for the stage settting.
            if self.blackboard["behavior_tree_on"] and not self.blackboard["stage_mode"]:
                print("----- Switch to stage mode")
                emo_scale /= self.blackboard["emotion_scale_closeup"]
                ges_scale /= self.blackboard["gesture_scale_closeup"]
            else:
                print("----- Behavior tree enabled, stage mode.")

            self.rescale_intensity(emo_scale, ges_scale)
            self.blackboard["stage_mode"] = True
            self.blackboard["behavior_tree_on"] = True

        elif data.data == "emotion_off":
            rospy.logwarn("emotion expression disabled)")
            self.do_pub_emotions = False

        elif data.data == "gesture_off":
            self.do_pub_gestures = False

        elif data.data == "btree_off":
            # Turn the head to neutral position
            self.facetrack.look_at_face(0)
            # Reset the variables that we sync in every cycle
            self.blackboard["face_targets"] = []
            self.blackboard["recognized_face_targets"] = []
            self.blackboard["talking_faces"] = []
            self.blackboard["blob_targets"] = []
            self.facetrack.visible_faces_blobs = []
            # Other flags
            self.blackboard["behavior_tree_on"] = False
            self.blackboard["is_interruption"] = True
            self.blackboard["stage_mode"] = False
            print("---- Behavior tree disabled")

    def chatbot_speech_end(self):
        # TODO
        # self.conversing=False
        print("---- speechend")

    # chatbot speech started, simulate listening
    # behavior tree will already cause face orient to speaker via web_ui
    def chatbot_speech_start(self):
        """
        Execute changes to blinks, saccades gestures, emotions to mimic conversational dynamics.
        """

        rospy.loginfo("webui starting speech")
        # do random choice from listening gestures
        self.pick_random_gesture("listening_gestures", "chat_perceived")
        # also nod with some probability

        if random.random()<self.blackboard["listening_nod_probability"]:
            min=self.blackboard["chatbot_positive_nod_speed_min"]
            max=self.blackboard["chatbot_positive_nod_speed_max"]
            speed=random.uniform(min,max)
            min=self.blackboard["chatbot_positive_nod_magnitude_min"]
            max=self.blackboard["chatbot_positive_nod_magnitude_max"]
            intensity=random.uniform(min,max)
            self.show_gesture("nod-6",intensity,1,speed,"chat_perceived")

        # switch to conversational (micro) saccade parameters
        msg = SaccadeCycle()
        msg.mean = self.blackboard["saccade_micro_interval_mean"]
        msg.variation = self.blackboard["saccade_micro_interval_var"]
        msg.paint_scale = self.blackboard["saccade_micro_paint_scale"]
        # from study face, maybe better default should be defined for explore
        msg.eye_size= self.blackboard["saccade_study_face_eye_size"]
        msg.eye_distance = self.blackboard["saccade_study_face_eye_distance"]
        msg.mouth_width =  self.blackboard["saccade_study_face_mouth_width"]

        msg.mouth_height = self.blackboard["saccade_study_face_mouth_height"]
        msg.weight_eyes = self.blackboard["saccade_study_face_weight_eyes"]
        msg.weight_mouth = self.blackboard["saccade_study_face_weight_mouth"]
        print 'switching to conversational microsaccade settings, mean= ', self.blackboard["saccade_micro_interval_mean"]
        self.saccade_pub.publish(msg)
        # TODO switch to conversational emotion set and probabilities
        # interaction will check this flag and choose alternate emotions
        self.conversing = True


    def chat_event_dispatcher_callback(self,chat_event):
        rospy.loginfo('chat_event, type '+chat_event.data)
        if chat_event.data=="speechstart":
            self.chatbot_speech_start()
        elif chat_event.data=="speechend":
            self.chatbot_speech_end()

    # chatbot requests blink
    def chatbot_blink_callback(self, blink):
        rospy.loginfo(blink.data +' says blink')
        blink_probabilities={'chat_heard':'chat_heard_probability',
                             'chat_saying':'chat_saying_probability',
                             'tts_end':'tts_end_probability'}
        # if we get a string not in the dictionary return 1.0
        blink_probability=self.blackboard[blink_probabilities[blink.data]]
        if random.random()<blink_probability:
            self.show_gesture('blink', 1.0, 1, 1.0, blink)

    # The perceived emotional content in the message.
    # emo is of type EmotionState
    def chatbot_affect_perceive_callback(self, emo):

        rospy.loginfo('chatbot perceived emo class ='+emo.data)
        # for now pass through to blender using random positive or non_positive class
        # in future we want more cognitive / behavior
        # pick random emotions may not do anything depending on random number so add force optional arg
        force=True

        # turn random emotion switch off to block emotion switching except from chat

        cached_pub_emotions = self.do_pub_emotions
        self.do_pub_emotions=False
        # pick gesture and expression functions require this trigger or may not work
        trigger='chat_perceived'
        if emo.data == 'happy':


            #chosen_emo=self.pick_random_expression("positive_emotions",trigger,force)
            rospy.loginfo('using neutral speech emotions expressions set')
            chosen_emo=self.pick_random_expression("neutral_speech_emotions",trigger,force)
            # change blink rate
            self.blink_update(self.blackboard["blink_chat_faster_mean"],0.12,True)
            # nod slowly with some probability
            if random.random()<self.blackboard["chatbot_positive_nod_probability"]:
                min=self.blackboard["chatbot_positive_nod_speed_min"]
                max=self.blackboard["chatbot_positive_nod_speed_max"]
                speed=random.uniform(min,max)
                min=self.blackboard["chatbot_positive_nod_magnitude_min"]
                max=self.blackboard["chatbot_positive_nod_magnitude_max"]
                intensity=random.uniform(min,max)
                rospy.loginfo('invoking nod for response')
                # TODO alternate affirmation gestures
                self.show_gesture('nod-6', intensity, 1,speed, trigger)
            # raise eyebrows with some probability
            if random.random()<self.blackboard["chatbot_positive_reply_think_probability"]:
                speed=random.uniform(0.3,0.5)
                intensity=random.uniform(0.5,0.7)
                rospy.loginfo('invoking positive think gesture')
                # TODO alternate think gestures
                self.show_gesture("think-browsUp",intensity,1,speed,trigger)
        else: # negative polarity
            # change blink rate
            self.blink_update(self.blackboard["blink_chat_slower_mean"],.12,True)
            chosen_emo=self.pick_random_expression("frustrated_emotions",trigger,force)
            if random.random()<self.blackboard["chatbot_negative_shake_probability"]:
                min=self.blackboard["chatbot_negative_shake_speed_min"]
                max=self.blackboard["chatbot_negative_shake_speed_max"]
                speed=random.uniform(min,max)
                min=self.blackboard["chatbot_negative_shake_magnitude_min"]
                max=self.blackboard["chatbot_negative_shake_magnitude_max"]
                intensity=random.uniform(min,max)
                rospy.loginfo('invoking shake for response')
                # TODO alternate shake gestures

                self.show_gesture('shake-3', intensity, 1,speed, trigger)
            # raise eyebrows with some probability
            # squint or furrow brows with some probability
            # TODO add probability test from config
            if random.random()<self.blackboard["chatbot_negative_reply_think_probability"]:
                speed=random.uniform(0.3,0.5)
                intensity=random.uniform(0.5,0.7)
                rospy.loginfo('invoking negative think gesture')
                self.show_gesture("think-browsDown.003",intensity,1,speed,trigger)

        # publish this message to cause chatbot to emit response if it's waiting
        exp = EmotionState()
        #  should now be consistent after setting show expression since = None upstream
        exp.name = self.blackboard["current_emotion"]
        #  TODO replace this hard coding with .cfg values. Neutral speech values are defined but not
        exp.magnitude = 0.8
        # use zero for duration, tts can compute if needed
        exp.duration.secs = 4.0
        exp.duration.nsecs = 0
        rospy.logwarn('publishing affect to chatbot '+chosen_emo.name)
        self.affect_pub.publish(exp)
        rospy.loginfo('picked and expressed '+chosen_emo.name)
        self.do_pub_emotions=cached_pub_emotions

    # reset blink rate
    def blink_update(self,mean,variation,reset=False):
        msg = BlinkCycle()
        msg.mean=mean
        msg.variation=variation
        self.blink_pub.publish(msg)

    # default rates when behavior turned on
    def init_blink(self):
        msg = BlinkCycle()
        msg.mean = self.blackboard["blink_randomly_interval_mean"]
        msg.variation = self.blackboard["blink_randomly_interval_var"]
        print 'initialize blink mean', self.blackboard["blink_randomly_interval_mean"]
        self.blink_pub.publish(msg)

    # default rates and face heat map config
    def init_saccade(self):
        msg = SaccadeCycle()
        msg.mean = self.blackboard["saccade_explore_interval_mean"]
        msg.variation = self.blackboard["saccade_explore_interval_var"]
        msg.paint_scale = self.blackboard["saccade_explore_paint_scale"]
        # from study face, maybe better default should be defined for explore
        #
        msg.eye_size= self.blackboard["saccade_study_face_eye_size"]
        msg.eye_distance = self.blackboard["saccade_study_face_eye_distance"]
        msg.mouth_width =  self.blackboard["saccade_study_face_mouth_width"]

        msg.mouth_height = self.blackboard["saccade_study_face_mouth_height"]
        msg.weight_eyes = self.blackboard["saccade_study_face_weight_eyes"]
        msg.weight_mouth = self.blackboard["saccade_study_face_weight_mouth"]
        self.saccade_pub.publish(msg)


