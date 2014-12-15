#! /usr/bin/env python
import owyl
from owyl import blackboard
import rospy
import roslib
import random
import time
import ConfigParser
import os
from std_msgs.msg import String
from eva_behavior.msg import event
from eva_behavior.msg import tracking_action
from basic_head_api.msg import MakeCoupledFaceExpr


class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard("rig expressions")

        ##### From Config File #####
        def get_values(from_config, num_of_values, is_probability):
            rtn_values = [float(z.strip()) for z in from_config.split(",")]
            if len(rtn_values) != num_of_values:
                raise Exception("Value counts don't match!")
            if is_probability and sum(rtn_values) != 1.0:
                raise Exception("Probabilities don't sum up to 1.0!")
            return rtn_values
        config = ConfigParser.ConfigParser()
        config.readfp(open(os.path.join(os.path.dirname(__file__), "../behavior.cfg")))
        self.blackboard["sadness_happiness"] = config.getfloat("emotion", "sadness_happiness")
        self.blackboard["irritation_amusement"] = config.getfloat("emotion", "irritation_amusement")
        self.blackboard["confusion_comprehension"] = config.getfloat("emotion", "confusion_comprehension")
        self.blackboard["boredom_engagement"] = config.getfloat("emotion", "boredom_engagement")
        self.blackboard["recoil_surprise"] = config.getfloat("emotion", "recoil_surprise")
        self.blackboard["emotions"] = [x.strip() for x in config.get("emotion", "basic_emotions").split(",")]
        self.blackboard["current_emotion"] = config.get("emotion", "default_emotion")
        self.blackboard["current_emotion_intensity"] = config.getfloat("emotion", "default_emotion_intensity")
        self.blackboard["frustrated_emotions"] = [x.strip() for x in config.get("emotion", "frustrated_emotions").split(",")]
        self.blackboard["frustrated_emotions_probabilities"] = get_values(config.get("emotion", "frustrated_emotions_probabilities"), len(self.blackboard["frustrated_emotions"]), True)
        self.blackboard["frustrated_emotions_intensities_min"] = get_values(config.get("emotion", "frustrated_emotions_intensities_min"), len(self.blackboard["frustrated_emotions"]), False)
        self.blackboard["frustrated_emotions_intensities_max"] = get_values(config.get("emotion", "frustrated_emotions_intensities_max"), len(self.blackboard["frustrated_emotions"]), False)
        self.blackboard["positive_emotions"] = [x.strip() for x in config.get("emotion", "positive_emotions").split(",")]
        self.blackboard["positive_emotions_probabilities"] = get_values(config.get("emotion", "positive_emotions_probabilities"), len(self.blackboard["positive_emotions"]), True)
        self.blackboard["positive_emotions_intensities_min"] = get_values(config.get("emotion", "positive_emotions_intensities_min"), len(self.blackboard["positive_emotions"]), False)
        self.blackboard["positive_emotions_intensities_max"] = get_values(config.get("emotion", "positive_emotions_intensities_max"), len(self.blackboard["positive_emotions"]), False)
        self.blackboard["show_expressions_other_than_positive_probabilities"] = config.getfloat("emotion", "show_expressions_other_than_positive_probabilities")
        self.blackboard["expressions_other_than_positive_intensity_min"] = config.getfloat("emotion", "expressions_other_than_positive_intensity_min")
        self.blackboard["expressions_other_than_positive_intensity_max"] = config.getfloat("emotion", "expressions_other_than_positive_intensity_max")
        self.blackboard["expressions_other_than_positive_duration_min"] = config.getfloat("emotion", "expressions_other_than_positive_duration_min")
        self.blackboard["expressions_other_than_positive_duration_max"] = config.getfloat("emotion", "expressions_other_than_positive_duration_max")
        self.blackboard["boring_emotions"] = [x.strip() for x in config.get("emotion", "boring_emotions").split(",")]
        self.blackboard["boring_emotions_probabilities"] = get_values(config.get("emotion", "boring_emotions_probabilities"), len(self.blackboard["boring_emotions"]), True)
        self.blackboard["boring_emotions_intensities_min"] = get_values(config.get("emotion", "boring_emotions_intensities_min"), len(self.blackboard["boring_emotions"]), False)
        self.blackboard["boring_emotions_intensities_max"] = get_values(config.get("emotion", "boring_emotions_intensities_max"), len(self.blackboard["boring_emotions"]), False)
        self.blackboard["show_expressions_other_than_boring_probabilities"] = config.getfloat("emotion", "show_expressions_other_than_boring_probabilities")
        self.blackboard["expressions_other_than_boring_intensity_min"] = config.getfloat("emotion", "expressions_other_than_boring_intensity_min")
        self.blackboard["expressions_other_than_boring_intensity_max"] = config.getfloat("emotion", "expressions_other_than_boring_intensity_max")
        self.blackboard["expressions_other_than_boring_duration_min"] = config.getfloat("emotion", "expressions_other_than_boring_duration_min")
        self.blackboard["expressions_other_than_boring_duration_max"] = config.getfloat("emotion", "expressions_other_than_boring_duration_max")
        self.blackboard["sleep_emotions"] = [x.strip() for x in config.get("emotion", "sleep_emotions").split(",")]
        self.blackboard["sleep_emotions_probabilities"] = get_values(config.get("emotion", "sleep_emotions_probabilities"), len(self.blackboard["sleep_emotions"]), True)
        self.blackboard["sleep_emotions_intensities_min"] = get_values(config.get("emotion", "sleep_emotions_intensities_min"), len(self.blackboard["sleep_emotions"]), False)
        self.blackboard["sleep_emotions_intensities_max"] = get_values(config.get("emotion", "sleep_emotions_intensities_max"), len(self.blackboard["sleep_emotions"]), False)
        self.blackboard["show_expressions_other_than_sleep_probabilities"] = config.getfloat("emotion", "show_expressions_other_than_sleep_probabilities")
        self.blackboard["expressions_other_than_sleep_intensity_min"] = config.getfloat("emotion", "expressions_other_than_sleep_intensity_min")
        self.blackboard["expressions_other_than_sleep_intensity_max"] = config.getfloat("emotion", "expressions_other_than_sleep_intensity_max")
        self.blackboard["expressions_other_than_sleep_duration_min"] = config.getfloat("emotion", "expressions_other_than_sleep_duration_min")
        self.blackboard["expressions_other_than_sleep_duration_max"] = config.getfloat("emotion", "expressions_other_than_sleep_duration_max")
        self.blackboard["wake_up_emotions"] = [x.strip() for x in config.get("emotion", "wake_up_emotions").split(",")]
        self.blackboard["wake_up_emotions_probabilities"] = get_values(config.get("emotion", "wake_up_emotions_probabilities"), len(self.blackboard["wake_up_emotions"]), True)
        self.blackboard["wake_up_emotions_intensities_min"] = get_values(config.get("emotion", "wake_up_emotions_intensities_min"), len(self.blackboard["wake_up_emotions"]), False)
        self.blackboard["wake_up_emotions_intensities_max"] = get_values(config.get("emotion", "wake_up_emotions_intensities_max"), len(self.blackboard["wake_up_emotions"]), False)
        self.blackboard["min_duration_for_interaction"] = config.getfloat("interaction", "duration_min")
        self.blackboard["max_duration_for_interaction"] = config.getfloat("interaction", "duration_max")
        self.blackboard["time_to_change_face_target_min"] = config.getfloat("interaction", "time_to_change_face_target_min")
        self.blackboard["time_to_change_face_target_max"] = config.getfloat("interaction", "time_to_change_face_target_max")
        self.blackboard["glance_probability"] = config.getfloat("interaction", "glance_probability")
        self.blackboard["glance_probability_for_new_faces"] = config.getfloat("interaction", "glance_probability_for_new_faces")
        self.blackboard["glance_probability_for_lost_faces"] = config.getfloat("interaction", "glance_probability_for_lost_faces")
        self.blackboard["sleep_probability"] = config.getfloat("boredom", "sleep_probability")
        self.blackboard["sleep_duration_min"] = config.getfloat("boredom", "sleep_duration_min")
        self.blackboard["sleep_duration_max"] = config.getfloat("boredom", "sleep_duration_max")
        self.blackboard["search_for_attention_duration_min"] = config.getfloat("boredom", "search_for_attention_duration_min")
        self.blackboard["search_for_attention_duration_max"] = config.getfloat("boredom", "search_for_attention_duration_max")
        self.blackboard["wake_up_probability"] = config.getfloat("boredom", "wake_up_probability")
        self.blackboard["time_to_wake_up"] = config.getfloat("boredom", "time_to_wake_up")

        ##### Other System Variables #####
        self.blackboard["show_expression_since"] = time.time()
        self.blackboard["face_targets"] = []  # IDs of faces in the scene, updated once per cycle
        self.blackboard["background_face_targets"] = []  # IDs of faces in the scene, updated immediately
        self.blackboard["new_face"] = ""
        self.blackboard["lost_face"] = ""
        self.blackboard["current_glance_target"] = ""
        self.blackboard["current_face_target"] = ""
        self.blackboard["interact_with_face_target_since"] = 0.0
        self.blackboard["sleep_since"] = 0.0
        self.blackboard["is_interruption"] = False
        self.blackboard["is_sleeping"] = False
        self.blackboard["blender_mode"] = ""
        self.blackboard["is_scripted_performance_system_on"] = True
        self.blackboard["random"] = 0.0

        ##### ROS Connections #####
        rospy.Subscriber("behavior_switch", String, self.behavior_switch_callback)
        rospy.Subscriber("/tracking_event", event, self.tracking_event_callback)
        self.tracking_mode_pub = rospy.Publisher("/cmd_blendermode", String, queue_size=1, latch=True)
        self.action_pub = rospy.Publisher("/tracking_action", tracking_action, queue_size=5, latch=True)
        self.emotion_pub = rospy.Publisher("/arthur/make_coupled_face_expr", MakeCoupledFaceExpr, queue_size=1)
        self.tree = self.build_tree()
        time.sleep(0.1)
        while True:
            self.tree.next()

    def build_tree(self):
        eva_behavior_tree = \
            owyl.repeatAlways(
                owyl.selector(
                    owyl.sequence(
                        self.is_scripted_performance_system_off(),
                        self.sync_variables(),
                        ########## Main Events ##########
                        owyl.selector(
                            ##### When Someone Arrived #####
                            owyl.sequence(
                                self.is_someone_arrived(),
                                self.set_emotion(variable="boredom_engagement", value=0.5),
                                owyl.selector(
                                    ##### Were No People In The Scene #####
                                    owyl.sequence(
                                        self.were_no_people_in_the_scene(),
                                        self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.2, max=1.4),
                                        self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.2, max=1.4),
                                        self.assign_face_target(variable="current_face_target", value="new_face"),
                                        self.record_start_time(variable="interact_with_face_target_since"),
                                        self.interact_with_face_target(id="current_face_target", new_face=True)
                                    ),

                                    ##### Is Interacting With Someone #####
                                    owyl.sequence(
                                        self.is_interacting_with_someone(),
                                        self.is_random_smaller_than(val1="newRandom", val2="glance_probability_for_new_faces"),
                                        self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.05, max=1.1),
                                        self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.05, max=1.1),
                                        self.glance_at_new_face()
                                    ),

                                    ##### Does Nothing #####
                                    owyl.sequence(
                                        self.print_status(str="----- Ignoring The New Face!"),
                                        self.does_nothing()
                                    )
                                ),
                                self.clear_new_face_target()
                            ),

                            ##### When Someone Left #####
                            owyl.sequence(
                                self.is_someone_left(),
                                owyl.selector(
                                    ##### Was Interacting With That Person #####
                                    owyl.sequence(
                                        self.was_interacting_with_that_person(),
                                        self.update_emotion(variable="confusion_comprehension", lower_limit=0.0, min=0.4, max=0.6),
                                        self.update_emotion(variable="recoil_surprise", lower_limit=0.0, min=1.8, max=2.2),
                                        self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.4, max=0.6),
                                        self.update_emotion(variable="irritation_amusement", lower_limit=0.0, min=0.95, max=1.0),
                                        self.show_frustrated_expression()
                                    ),

                                    ##### Is Interacting With Someone Else #####
                                    owyl.sequence(
                                        self.is_interacting_with_someone(),
                                        self.is_random_smaller_than(val1="newRandom", val2="glance_probability_for_lost_faces"),
                                        self.glance_at_lost_face()
                                    ),

                                    ##### Does Nothing #####
                                    owyl.sequence(
                                        self.print_status(str="----- Ignoring The Lost Face!"),
                                        self.does_nothing()
                                    )
                                ),
                                self.clear_lost_face_target()
                            ),

                            ##### People Interaction #####
                            owyl.sequence(
                                self.is_face_target(),
                                self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.001, max=1.005),
                                self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.005, max=1.01),
                                owyl.selector(
                                    ##### Start A New Interaction #####
                                    owyl.sequence(
                                        owyl.selector(
                                            self.is_not_interacting_with_someone(),
                                            owyl.sequence(
                                                self.is_more_than_one_face_target(),
                                                self.is_time_to_change_face_target()
                                            )
                                        ),
                                        self.select_a_face_target(),
                                        self.record_start_time(variable="interact_with_face_target_since"),
                                        self.interact_with_face_target(id="current_face_target", new_face=False)
                                    ),

                                    ##### Glance At Other Faces & Continue With The Last Interaction #####
                                    owyl.sequence(
                                        self.print_status(str="----- Continue The Interaction"),
                                        owyl.selector(
                                            owyl.sequence(
                                                self.is_more_than_one_face_target(),
                                                self.is_random_smaller_than(val1="newRandom", val2="glance_probability"),
                                                self.select_a_glance_target(),
                                                self.glance_at(id="current_glance_target")
                                            ),
                                            self.does_nothing()
                                        ),
                                        self.interact_with_face_target(id="current_face_target", new_face=False)
                                    )
                                )
                            ),

                            ##### Nothing Interesting Is Happening #####
                            owyl.sequence(
                                self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=0.8, max=0.9),
                                self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.995, max=1.0),
                                owyl.selector(
                                    ##### Is Not Sleeping #####
                                    owyl.sequence(
                                        self.is_not_sleeping(),
                                        owyl.selector(
                                            ##### Go To Sleep #####
                                            owyl.sequence(
                                                self.is_random_smaller_than(val1="newRandom_plus_boredom", val2="sleep_probability"),
                                                self.record_start_time(variable="sleep_since"),
                                                self.print_status(str="----- Go To Sleep!"),
                                                self.go_to_sleep()
                                            ),

                                            ##### Search For Attention #####
                                            self.search_for_attention()
                                        )
                                    ),

                                    ##### Is Sleeping #####
                                    owyl.selector(
                                        ##### Wake Up #####
                                        owyl.sequence(
                                            self.is_random_smaller_than(val1="newRandom", val2="wake_up_probability"),
                                            self.is_time_to_wake_up(),
                                            self.wake_up(),
                                            self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
                                        ),

                                        ##### Continue To Sleep #####
                                        owyl.sequence(
                                            self.print_status(str="----- Continue To Sleep!"),
                                            self.go_to_sleep()
                                        )
                                    )
                                ),

                                ##### If Interruption && Sleeping -> Wake Up #####
                                owyl.sequence(
                                    self.is_interruption(),
                                    self.is_sleeping(),
                                    self.wake_up(),
                                    self.print_status(str="----- Interruption: Wake Up!"),
                                    self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
                                )
                            )
                        )
                    ),

                    ############### Scripted Performance System ###############
                    owyl.sequence(
                        self.is_scripted_performance_system_on(),
                        self.start_scripted_performance_system()
                    )
                )
            )
        return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

    @owyl.taskmethod
    def print_status(self, **kwargs):
        print kwargs["str"]
        yield True

    @owyl.taskmethod
    def sync_variables(self, **kwargs):
        self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
        print "\n========== Emotion Space =========="
        print "sadness_happiness: " + str(self.blackboard["sadness_happiness"])[:5]
        print "irritation_amusement: " + str(self.blackboard["irritation_amusement"])[:5]
        print "confusion_comprehension: " + str(self.blackboard["confusion_comprehension"])[:5]
        print "boredom_engagement: " + str(self.blackboard["boredom_engagement"])[:5]
        print "recoil_surprise: " + str(self.blackboard["recoil_surprise"])[:5]
        print "Current Emotion: " + self.blackboard["current_emotion"] + " (" + str(self.blackboard["current_emotion_intensity"])[:5] + ")"
        yield True

    @owyl.taskmethod
    def does_nothing(self, **kwargs):
        yield True

    @owyl.taskmethod
    def set_emotion(self, **kwargs):
        self.blackboard[kwargs["variable"]] = kwargs["value"]
        yield True

    @owyl.taskmethod
    def update_emotion(self, **kwargs):
        if kwargs["lower_limit"] > 0.0:
            self.blackboard[kwargs["variable"]] = kwargs["lower_limit"]
        self.blackboard[kwargs["variable"]] *= random.uniform(kwargs["min"], kwargs["max"])
        if self.blackboard[kwargs["variable"]] > 1.0:
            self.blackboard[kwargs["variable"]] = 1.0
        elif self.blackboard[kwargs["variable"]] <= 0.0:
            self.blackboard[kwargs["variable"]] = 0.01
        yield True

    @owyl.taskmethod
    def is_random_smaller_than(self, **kwargs):
        if kwargs["val1"] == "newRandom":
            self.blackboard["random"] = random.random()
        elif kwargs["val1"] == "newRandom_plus_boredom":
            self.blackboard["random"] = random.random() + self.blackboard["boredom_engagement"]
        if self.blackboard["random"] < self.blackboard[kwargs["val2"]]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_arrived(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["new_face"]:
            print "----- Someone Arrived!"
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_left(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if len(self.blackboard["lost_face"]) > 0:
            print "----- Someone Left!"
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
    def were_no_people_in_the_scene(self, **kwargs):
        if len(self.blackboard["face_targets"]) == 1:
            print "----- Were No People In The Scene!"
            yield True
        else:
            yield False

    @owyl.taskmethod
    def was_interacting_with_that_person(self, **kwargs):
        if self.blackboard["current_face_target"] == self.blackboard["lost_face"]:
            self.blackboard["current_face_target"] = ""
            print "----- Was Interacting With That Person!"
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
    def is_more_than_one_face_target(self, **kwargs):
        if len(self.blackboard["face_targets"]) > 1:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_time_to_change_face_target(self, **kwargs):
        if self.blackboard["interact_with_face_target_since"] > 0 and (time.time() - self.blackboard["interact_with_face_target_since"]) >= random.uniform(self.blackboard["time_to_change_face_target_min"], self.blackboard["time_to_change_face_target_max"]):
            print "----- Time To Start A New Interaction!"
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
    def is_scripted_performance_system_on(self, **kwargs):
        if self.blackboard["is_scripted_performance_system_on"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_scripted_performance_system_off(self, **kwargs):
        if not self.blackboard["is_scripted_performance_system_on"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def assign_face_target(self, **kwargs):
        self.blackboard[kwargs["variable"]] = self.blackboard[kwargs["value"]]
        yield True

    @owyl.taskmethod
    def select_a_face_target(self, **kwargs):
        self.blackboard["current_face_target"] = random.choice(self.blackboard["face_targets"])
        yield True

    @owyl.taskmethod
    def select_a_glance_target(self, **kwargs):
        target = random.choice(self.blackboard["face_targets"])
        while target == self.blackboard["current_face_target"]:
            target = random.choice(self.blackboard["face_targets"])
        self.blackboard["current_glance_target"] = target
        yield True

    @owyl.taskmethod
    def record_start_time(self, **kwargs):
        self.blackboard[kwargs["variable"]] = time.time()
        yield True

    @owyl.taskmethod
    def interact_with_face_target(self, **kwargs):
        if self.blackboard["blender_mode"] != "TrackDev":
            self.tracking_mode_pub.publish("TrackDev")
            self.blackboard["blender_mode"] = "TrackDev"
            time.sleep(0.1)
        face_id = self.blackboard[kwargs["id"]]
        duration = random.uniform(self.blackboard["min_duration_for_interaction"], self.blackboard["max_duration_for_interaction"])
        interval = 0.01
        action = tracking_action()
        action.target = "/faces/" + face_id
        action.action = "track"
        action.params = ""
        self.action_pub.publish(action)
        print "----- Is Interacting With Face(id:" + face_id + ") for " + str(duration)[:5] + " seconds"
        if time.time() - self.blackboard["show_expression_since"] >= 2.0 or kwargs["new_face"]:
            ##### Show A Random Instant Expression #####
            if random.random() <= self.blackboard["show_expressions_other_than_positive_probabilities"]:
                self.show(random.choice([other for other in self.blackboard["emotions"] if other not in self.blackboard["positive_emotions"]]), random.uniform(self.blackboard["expressions_other_than_positive_intensity_min"], self.blackboard["expressions_other_than_positive_intensity_max"]))
                time.sleep(random.uniform(self.blackboard["expressions_other_than_positive_duration_min"], self.blackboard["expressions_other_than_positive_duration_max"]))
            ##### Show A Positive Expression #####
            sum = 0
            random_number = random.random()
            for i in range(0, len(self.blackboard["positive_emotions_probabilities"])):
                sum += self.blackboard["positive_emotions_probabilities"][i]
                if random_number <= sum:
                    expression_to_show = self.blackboard["positive_emotions"][i]
                    intensity_min = self.blackboard["positive_emotions_intensities_min"][i]
                    intensity_max = self.blackboard["positive_emotions_intensities_max"][i]
                    break
            self.show(expression_to_show, random.uniform(intensity_min, intensity_max))
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        yield True

    @owyl.taskmethod
    def glance_at(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        action = tracking_action()
        action.target = "/faces/" + face_id
        action.action = "glance"
        action.params = ""
        self.action_pub.publish(action)
        print "----- Glancing At Face(id:" + face_id + ")"
        yield True

    @owyl.taskmethod
    def glance_at_new_face(self, **kwargs):
        face_id = self.blackboard["new_face"]
        action = tracking_action()
        action.target = "/faces/" + face_id
        action.action = "glance"
        action.params = ""
        self.action_pub.publish(action)
        print "----- Glancing At The New Face(id:" + face_id + ")"
        yield True

    @owyl.taskmethod
    def glance_at_lost_face(self, **kwargs):
        print "----- Glancing At The Lost Face(id:" + self.blackboard["lost_face"] + ")"
        face_id = self.blackboard["lost_face"]
        action = tracking_action()
        action.target = "/faces/" + face_id
        action.action = "glance"
        action.params = ""
        self.action_pub.publish(action)
        yield True

    @owyl.taskmethod
    def show_expression(self, **kwargs):
        self.show(kwargs["expression"], random.uniform(kwargs["min_intensity"], kwargs["max_intensity"]))
        yield True

    @owyl.taskmethod
    def show_frustrated_expression(self, **kwargs):
        random_number = random.random()
        sum = 0
        for i in range(0, len(self.blackboard["frustrated_emotions_probabilities"])):
            sum += self.blackboard["frustrated_emotions_probabilities"][i]
            if random_number <= sum:
                expression_to_show = self.blackboard["frustrated_emotions"][i]
                intensity_min = self.blackboard["frustrated_emotions_intensities_min"][i]
                intensity_max = self.blackboard["frustrated_emotions_intensities_max"][i]
                break
        self.show(expression_to_show, random.uniform(intensity_min, intensity_max))
        yield True

    def show(self, expression, intensity):
        exp = MakeCoupledFaceExpr()
        exp.robotname = "arthur"
        self.blackboard["current_emotion"] = expression
        self.blackboard["current_emotion_intensity"] = intensity
        exp.expr.exprname = self.blackboard["current_emotion"]
        exp.expr.intensity = self.blackboard["current_emotion_intensity"]
        self.emotion_pub.publish(exp)
        print "----- Show expression: " + expression + " (" + str(intensity)[:5] + ")"
        if self.blackboard["current_emotion"] == "surprise":
            exp.expr.intensity = 0.2
            self.emotion_pub.publish(exp)
        self.blackboard["show_expression_since"] = time.time()

    @owyl.taskmethod
    def search_for_attention(self, **kwargs):
        print "----- Search For Attention!"
        duration = random.uniform(self.blackboard["search_for_attention_duration_min"], self.blackboard["search_for_attention_duration_max"])
        interval = 0.01
        if self.blackboard["blender_mode"] != "LookAround":
            self.tracking_mode_pub.publish("LookAround")
            self.blackboard["blender_mode"] = "LookAround"
        if time.time() - self.blackboard["show_expression_since"] >= 5.0:
            ##### Show A Random Instant Expression #####
            if random.random() <= self.blackboard["show_expressions_other_than_boring_probabilities"]:
                self.show(random.choice([other for other in self.blackboard["emotions"] if other not in self.blackboard["boring_emotions"]]), random.uniform(self.blackboard["expressions_other_than_boring_intensity_min"], self.blackboard["expressions_other_than_boring_intensity_max"]))
                time.sleep(random.uniform(self.blackboard["expressions_other_than_boring_duration_min"], self.blackboard["expressions_other_than_boring_duration_max"]))
            ##### Show A Positive Expression #####
            sum = 0
            random_number = random.random()
            for i in range(0, len(self.blackboard["boring_emotions_probabilities"])):
                sum += self.blackboard["boring_emotions_probabilities"][i]
                if random_number <= sum:
                    expression_to_show = self.blackboard["boring_emotions"][i]
                    intensity_min = self.blackboard["boring_emotions_intensities_min"][i]
                    intensity_max = self.blackboard["boring_emotions_intensities_max"][i]
                    break
            self.show(expression_to_show, random.uniform(intensity_min, intensity_max))
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        yield True

    @owyl.taskmethod
    def go_to_sleep(self, **kwargs):
        self.blackboard["is_sleeping"] = True
        ##### Show A Sleep Expression #####
        sum = 0
        random_number = random.random()
        for i in range(0, len(self.blackboard["sleep_emotions_probabilities"])):
            sum += self.blackboard["sleep_emotions_probabilities"][i]
            if random_number <= sum:
                expression_to_show = self.blackboard["sleep_emotions"][i]
                intensity_min = self.blackboard["sleep_emotions_intensities_min"][i]
                intensity_max = self.blackboard["sleep_emotions_intensities_max"][i]
                break
        self.show(expression_to_show, random.uniform(intensity_min, intensity_max))
        duration = random.uniform(self.blackboard["sleep_duration_min"], self.blackboard["sleep_duration_max"])
        interval = 0.01
        #TODO: Topic for sleep
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        yield True

    @owyl.taskmethod
    def wake_up(self, **kwargs):
        print "----- Wake Up!"
        self.blackboard["is_sleeping"] = False
        self.blackboard["sleep_since"] = 0.0
        ##### Show A Wake Up Expression #####
        sum = 0
        random_number = random.random()
        for i in range(0, len(self.blackboard["wake_up_emotions_probabilities"])):
            sum += self.blackboard["wake_up_emotions_probabilities"][i]
            if random_number <= sum:
                expression_to_show = self.blackboard["wake_up_emotions"][i]
                intensity_min = self.blackboard["wake_up_emotions_intensities_min"][i]
                intensity_max = self.blackboard["wake_up_emotions_intensities_max"][i]
                break
        self.show(expression_to_show, random.uniform(intensity_min, intensity_max))
        #TODO: Topic for waking up
        yield True

    @owyl.taskmethod
    def clear_new_face_target(self, **kwargs):
        if not self.blackboard["is_interruption"]:
            print "----- Cleared New Face: " + self.blackboard["new_face"]
            self.blackboard["new_face"] = ""
        yield True

    @owyl.taskmethod
    def clear_lost_face_target(self, **kwargs):
        print "----- Cleared Lost Face: " + self.blackboard["lost_face"]
        self.blackboard["lost_face"] = ""
        yield True

    @owyl.taskmethod
    def start_scripted_performance_system(self, **kwargs):
        if self.blackboard["blender_mode"] != "Dummy":
            # No need to set Dummy mode
            #self.tracking_mode_pub.publish("Dummy")
            self.blackboard["blender_mode"] = "Dummy"
        yield True

    def tracking_event_callback(self, data):
        self.blackboard["is_interruption"] = True
        if data.event == "new_face":
            print "<< Interruption >> New Face Detected: " + data.param
            self.blackboard["new_face"] = data.param
            self.blackboard["background_face_targets"].append(self.blackboard["new_face"])
        elif data.event == "exit":
            print "<< Interruption >> Lost Face Detected: " + data.param
            if data.param in self.blackboard["background_face_targets"]:
                self.blackboard["lost_face"] = data.param
                self.blackboard["background_face_targets"].remove(self.blackboard["lost_face"])
                # If the robot lost the new face during the initial interaction, reset new_face variable
                if self.blackboard["new_face"] == self.blackboard["lost_face"]:
                    self.blackboard["new_face"] = ""

    def behavior_switch_callback(self, data):
        if data.data == "btree_on":
            self.blackboard["is_interruption"] = False
            self.blackboard["is_scripted_performance_system_on"] = False
            print "Behavior Tree is ON!"
        elif data.data == "btree_off":
            self.blackboard["is_interruption"] = True
            self.blackboard["is_scripted_performance_system_on"] = True
            print "Behavior Tree is OFF!"

if __name__ == "__main__":
    rospy.init_node("Eva_Behavior")
    tree = Tree()
